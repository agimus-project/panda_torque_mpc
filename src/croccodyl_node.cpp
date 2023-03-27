#incldue < string>
#include "ros/ros.h"

#include "panda_torque_mpc/common.h"

#include "croccodyl_reaching.h"

#include <linear_feedback_controller_msgs/Sensor.h>
#include <linear_feedback_controller_msgs/Control.h>
#include <linear_feedback_controller_msgs/eigen_conversions.hpp>

class CrocoddylNode
{

    CrocoddylNode(ros::NodeHandle &nh, std::string sensor_sub_topic, std::string control_pub_topic, std::string ref_sub_topic)
    {

        ///////////////////
        // Load parameters
        ///////////////////
        std::string arm_id;
        if (!get_param_error_tpl<std::string>(nh, arm_id, "arm_id"))
            return false;

        // Croco params
        int nb_shooting_nodes;
        double dt_ocp, w_frame_running, w_frame_terminal, w_x_reg_running, w_x_reg_terminal, scale_q_reg, w_u_reg_running;
        std::vector<double> armature, diag_u_reg_running;

        if (!get_param_error_tpl<int>(nh, nb_shooting_nodes, "nb_shooting_nodes"))
            return false;
        if (!get_param_error_tpl<double>(nh, dt_ocp, "dt_ocp"))
            return false;
        if (!get_param_error_tpl<double>(nh, w_frame_running, "w_frame_running"))
            return false;
        if (!get_param_error_tpl<double>(nh, w_frame_terminal, "w_frame_terminal"))
            return false;
        if (!get_param_error_tpl<double>(nh, w_x_reg_running, "w_x_reg_running"))
            return false;
        if (!get_param_error_tpl<double>(nh, w_x_reg_terminal, "w_x_reg_terminal"))
            return false;
        if (!get_param_error_tpl<double>(nh, scale_q_reg, "scale_q_reg"))
            return false;
        if (!get_param_error_tpl<double>(nh, w_u_reg_running, "w_u_reg_running"))
            return false;

        if (!get_param_error_tpl<std::vector<double>>(nh, armature, "armature",
                                                      [](std::vector<double> v)
                                                      { return v.size() == 7; }))
            return false;
        if (!get_param_error_tpl<std::vector<double>>(nh, diag_u_reg_running, "diag_u_reg_running",
                                                      [](std::vector<double> v)
                                                      { return v.size() == 7; }))
            return false;

        // Load panda model with pinocchio
        std::string urdf_path;
        if (!get_param_error_tpl<std::string>(nh, urdf_path, "urdf_path"))
            return false;

        /////////////////////////////////////////////////
        //                 Pinocchio                   //
        /////////////////////////////////////////////////
        pin::urdf::buildModel(urdf_path, model_pin_);
        std::cout << "model name: " << model_pin_.name << std::endl;
        data_pin_ = pin::Data(model_pin_);

        if ((model_pin_.nq != 7) || (model_pin_.name != "panda"))
        {
            ROS_ERROR_STREAM("Problem when loading the robot urdf");
            return false;
        }

        // Define corresponding frame id for pinocchio and Franka (see ctrl_model_pinocchio_vs_franka)
        ee_frame_pin_ = "panda_link8";
        ee_frame_id_ = model_pin_.getFrameId(ee_frame_pin_);

        /////////////////////////////////////////////////
        //                MPC CONFIG                   //
        /////////////////////////////////////////////////
        config_croco_.T = nb_shooting_nodes;
        config_croco_.dt_ocp = dt_ocp;
        config_croco_.ee_frame_name = ee_frame_pin_;
        config_croco_.w_frame_running = w_frame_running;
        config_croco_.w_frame_terminal = w_frame_terminal;
        config_croco_.w_x_reg_running = w_x_reg_running;
        config_croco_.w_x_reg_terminal = w_x_reg_terminal;
        config_croco_.scale_q_reg = scale_q_reg;
        config_croco_.w_u_reg_running = w_u_reg_running;
        config_croco_.armature = Eigen::Map<Eigen::Matrix<double, 7, 1>>(armature.data());
        config_croco_.diag_u_reg_running = Eigen::Map<Eigen::Matrix<double, 7, 1>>(diag_u_reg_running.data());

        croco_reaching_ = CrocoddylReaching(model_pin_, config_croco_);
        /////////////////////////////////////////////////

        // Publisher/Subscriber
        control_pub_ = n.advertise<linear_feedback_controller_msgs::Control>(control_pub_topic, 1);
        ros::Subscriber sensor_sub = n.subscribe(sensor_sub_topic, 1, &CrocoddylNode::callback_sensor, this);
        ros::Subscriber ref_sub = n.subscribe(ref_sub_topic, 1, &CrocoddylNode::callback_ref, this);
        
    }

    void callback_ref(const geometry_msgs::Pose &pose_msg)
    {
        Eigen::Vector3d position; 
        position << pose_msg.position.x, pose_msg.position.y, pose_msg.position.z;
        croco_reaching_.set_goal_translation(position);
    }

    void callback_sensor(const linear_feedback_controller_msgs::Sensor &sensor_msg)
    {
        // Separate callback for reference?

        // Recover latest robot state from the sensor msg
        linear_feedback_controller_msgs::Eigen::JointState sensor_eig;
        linear_feedback_controller_msgs::Eigen::sensorMsgToEigen(sensor_msg, sensor_eig);
        // TODO: Protect by a mutex!
        current_x_ << sensor_eig.position, sensor_eig.velocity;
    }

    void solve_and_send()
    {
        // Do nothing if no goal has been set yet
        if (!croco_reaching_.goal_translation_set_)
        {
            return;
        }

        // TODO: set gravity to zero instead
        VectorXd q = current_x_.head(model_pin_.nq());
        // Use VectorXd instead?
        Vector7d tau_grav = pin::computeGeneralizedGravity(model_pin_, data_pin_, q);

        // Update DDP problem
        // - shift trajectory by ??
        //   -> depends on node freq and node dt
        // - update current state with


        std::vector<Eigen::Matrix<double, -1, 1>> xs_init;
        std::vector<Eigen::Matrix<double, -1, 1>> us_init;

        if (first_time)
        {
            // if first occurence, use a sensible prior (no movement and gravity compensation)
            xs_init.reserve(config_croco_.T);
            us_init.reserve(config_croco_.T);
            for (int i = 0; i < config_croco_.T; i++)
            {
                xs_init.push_back(x0);
                us_init.push_back(tau_grav);
            }
        }
        else
        {
            // !!!!!!! //
            // HYP: dt_ocp == freq_node
            xs_init = croco_reaching_.ddp_->get_xs();
            us_init = croco_reaching_.ddp_->get_us();

            
            xs_init.insert(std::begin(xs_init), xs_init.at(0));
            xs_init.erase( std::end(xs_init)-1);
            us_init.insert(std::begin(us_init), us_init.at(0));
            us_init.erase( std::end(us_init)-1);
        }

        croco_reaching_.ddp_->solve(xs_init, us_init, nb_iterations_, false);
        Vector7d tau_d = croco_reaching_.ddp_->get_us()[0] - tau_grav;
        //////////////////////////////////////

        linear_feedback_controller_msgs::Eigen::Control ctrl_eig;
        // TODO: Fill with ctrl_eig dpp solve result

        // Fill and send control message
        linear_feedback_controller_msgs::Control ctrl_msg;
        linear_feedback_controller_msgs::Eigen::controlEigenToMsg(ctrl_eig, ctrl_msg);
        control_pub_.publish(ctrl_msg);
    }

    VectorXd current_x_;

    // Pinocchio objects
    pin::Model model_pin_;
    pin::Data data_pin_;
    std::string ee_frame_pin_;

    // MPC formulation
    CrocoddylReaching croco_reaching_;
    CrocoddylConfig config_croco_;

    // Publisher of commands
    ros::Publisher control_pub_;

}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "croccodyl_node");
    ros::NodeHandle n;
    std::string sensor_sub_topic = "sensor_sub_topic";
    std::string control_pub_topic = "control_pub_topic";
    std::string ref_sub_topic = "ee_pose_ref";
    auto node = CrocoddylNode(n, sensor_sub_topic, control_pub_topic, ref_sub_topic);

    int freq = 100;
    ros::Rate loop_rate(freq);

    while (ros::ok())
    {
        node.solve_and_send();
        ros::spinOnce();
        loop_rate.sleep();
    }
}