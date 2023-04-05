#include <string>

#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/rnea.hpp>

#include <linear_feedback_controller_msgs/Sensor.h>
#include <linear_feedback_controller_msgs/Control.h>
#include <linear_feedback_controller_msgs/eigen_conversions.hpp>

#include <realtime_tools/realtime_box.h>
#include <ros/ros.h>

#include "panda_torque_mpc/common.h"
#include "panda_torque_mpc/crocoddyl_reaching.h"

#include "panda_torque_mpc/PoseTaskGoal.h"

namespace panda_torque_mpc
{
    namespace lfc_msgs = linear_feedback_controller_msgs;

    class CrocoMotionServer
    {
    public:
        CrocoMotionServer(ros::NodeHandle &nh,
                          std::string robot_sensors_topic_sub,
                          std::string motion_server_control_topic_pub,
                          std::string ee_pose_ref_topic_sub)
        {
            bool params_success = true;
            ///////////////////
            // Load parameters
            ///////////////////
            std::string arm_id;
            params_success = get_param_error_tpl<std::string>(nh, arm_id, "arm_id") && params_success;

            // Croco params
            int nb_shooting_nodes;
            double dt_ocp, w_frame_running, w_frame_terminal, w_x_reg_running, w_x_reg_terminal, scale_q_vs_v_reg, w_u_reg_running;
            std::vector<double> armature, diag_u_reg_running;

            params_success = get_param_error_tpl<int>(nh, nb_shooting_nodes, "nb_shooting_nodes") && params_success;
            params_success = get_param_error_tpl<double>(nh, dt_ocp, "dt_ocp") && params_success;
            params_success = get_param_error_tpl<double>(nh, w_frame_running, "w_frame_running") && params_success;
            params_success = get_param_error_tpl<double>(nh, w_frame_terminal, "w_frame_terminal") && params_success;
            params_success = get_param_error_tpl<double>(nh, w_x_reg_running, "w_x_reg_running") && params_success;
            params_success = get_param_error_tpl<double>(nh, w_x_reg_terminal, "w_x_reg_terminal") && params_success;
            params_success = get_param_error_tpl<double>(nh, scale_q_vs_v_reg, "scale_q_vs_v_reg") && params_success;
            params_success = get_param_error_tpl<double>(nh, w_u_reg_running, "w_u_reg_running") && params_success;

            params_success = get_param_error_tpl<std::vector<double>>(nh, armature, "armature",
                                                                      [](std::vector<double> v)
                                                                      { return v.size() == 7; }) &&
                             params_success;
            params_success = get_param_error_tpl<std::vector<double>>(nh, diag_u_reg_running, "diag_u_reg_running",
                                                                      [](std::vector<double> v)
                                                                      { return v.size() == 7; }) &&
                             params_success;

            // Load panda model with pinocchio
            std::string urdf_path;
            params_success = get_param_error_tpl<std::string>(nh, urdf_path, "urdf_path") && params_success;

            if (!params_success)
            {
                throw std::invalid_argument("CrocoMotionServer: check the your ROS parameters");
            }

            /////////////////////////////////////////////////
            //                 Pinocchio                   //
            /////////////////////////////////////////////////
            pin::urdf::buildModel(urdf_path, model_pin_);
            std::cout << "model name: " << model_pin_.name << std::endl;
            data_pin_ = pin::Data(model_pin_);

            if ((model_pin_.nq != 7) || (model_pin_.name != "panda"))
            {
                ROS_ERROR_STREAM("Problem when loading the robot urdf");
                throw std::invalid_argument("CrocoMotionServer: Problem with the loaded robot model");
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
            config_croco_.scale_q_vs_v_reg = scale_q_vs_v_reg;
            config_croco_.w_u_reg_running = w_u_reg_running;
            config_croco_.armature = Eigen::Map<Eigen::Matrix<double, 7, 1>>(armature.data());
            config_croco_.diag_u_reg_running = Eigen::Map<Eigen::Matrix<double, 7, 1>>(diag_u_reg_running.data());

            croco_reaching_ = CrocoddylReaching(model_pin_, config_croco_);
            /////////////////////////////////////////////////

            // Publisher/Subscriber
            control_pub_ = nh.advertise<lfc_msgs::Control>(motion_server_control_topic_pub, 1);
            sensor_sub_ = nh.subscribe(robot_sensors_topic_sub, 10, &CrocoMotionServer::callback_sensor, this);
            pose_ref_sub_ = nh.subscribe(ee_pose_ref_topic_sub, 10, &CrocoMotionServer::callback_pose_ref, this);

            // Init some variables
            first_sensor_msg_received_ = false;
            first_pose_ref_msg_received_ = false;
            first_solve_ = true;
        }

        void callback_pose_ref(const PoseTaskGoal &msg)
        {
            /**
             * If the first sensor state of the robot has not yet been received, no need to process the pose ref
             */

            if (!first_sensor_msg_received_)
            {
                return;
            }

            Eigen::Vector3d p_bt;
            p_bt << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;
            Eigen::Quaterniond quap_bt(msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z);
            pin::SE3 T_w_t(quap_bt, p_bt);

            if (!first_pose_ref_msg_received_)
            {
                T_w_t0_ = T_w_t;
                first_pose_ref_msg_received_ = true;
            }

            // Cf ctrl_task_space_ID for instance for why this choice
            pin::SE3 T_e0_e = pin::SE3::Identity();
            auto R_e0_b = T_b_e0_.rotation().transpose();
            T_e0_e.translation() = R_e0_b * (T_w_t.translation() - T_w_t0_.translation());

            // Set reference pose
            // compose initial pose with relative/local transform
            pin::SE3 T_be = T_b_e0_ * T_e0_e;

            // RT safe setting
            T_be_rtbox_.set(T_be);
        }

        void callback_sensor(const lfc_msgs::Sensor &sensor_msg)
        {
            // Recover latest robot state from the sensor msg
            lfc_msgs::Eigen::Sensor sensor_eig;
            lfc_msgs::sensorMsgToEigen(sensor_msg, sensor_eig);
            // TODO: Protect by a mutex!
            Eigen::Matrix<double, 14, 1> current_x;
            current_x << sensor_eig.joint_state.position, sensor_eig.joint_state.velocity;
            current_x_rtbox_.set(current_x);

            // Separate callback for reference?
            if (!first_sensor_msg_received_)
            {
                pin::forwardKinematics(model_pin_, data_pin_, sensor_eig.joint_state.position);
                pin::updateFramePlacements(model_pin_, data_pin_);
                T_b_e0_ = data_pin_.oMf[ee_frame_id_];

                q_init_rtbox_.set(sensor_eig.joint_state.position);
                first_sensor_msg_received_ = true;
            }
        }

        void solve_and_send()
        {
            // Do nothing if no pose reference and sensor state has been received
            if (!(first_sensor_msg_received_ && first_pose_ref_msg_received_))
            {
                return;
            }

            // Retrieve reference in thread-safe way
            pin::SE3 T_be;
            T_be_rtbox_.get(T_be);

            // Retrieve initial configuration in thread-safe way
            Vector7d q_init;
            q_init_rtbox_.get(q_init);
            Eigen::Matrix<double,14,1> x_init; 
            x_init << q_init, Vector7d::Zero();  // Fix zero velocity as reference

            // Retrieve current state in a thread-safe way
            Eigen::Matrix<double, 14, 1> current_x;
            current_x_rtbox_.get(current_x);

            Vector7d q = current_x.head(model_pin_.nq);
            Vector7d v = current_x.tail(model_pin_.nv);

            std::vector<Eigen::Matrix<double, -1, 1>> xs_init;
            std::vector<Eigen::Matrix<double, -1, 1>> us_init;

            if (first_solve_)
            {
                // Warm start with gravity compensation control term
                Vector7d tau_grav = pin::computeGeneralizedGravity(model_pin_, data_pin_, q);

                // if first occurence, use a sensible prior (no movement and gravity compensation)
                xs_init.reserve(config_croco_.T);
                us_init.reserve(config_croco_.T);
                for (int i = 0; i < config_croco_.T; i++)
                {
                    xs_init.push_back(current_x);
                    us_init.push_back(tau_grav);
                }
                xs_init.push_back(current_x);

                first_solve_ = false;
            }
            else
            {
                // Warm start with previous solution shifted
                xs_init = croco_reaching_.ddp_->get_xs();
                us_init = croco_reaching_.ddp_->get_us();

                // Shift trajectory by 1 node <==> config_croco_.dt_ocp
                // !!!!!!! //
                // HYP: config_croco_.dt_ocp == freq_node
                // xs_init.insert(std::begin(xs_init), current_x);
                xs_init.insert(std::begin(xs_init), xs_init[0]);
                xs_init.erase(std::end(xs_init) - 1);
                us_init.insert(std::begin(us_init), us_init[0]);
                us_init.erase(std::end(us_init) - 1);
            }

            // Set initial state and end-effector ref
            croco_reaching_.ddp_->get_problem()->set_x0(current_x);
            croco_reaching_.set_ee_ref(T_be.translation());
            croco_reaching_.set_posture_ref(x_init);

            croco_reaching_.ddp_->solve(xs_init, us_init, config_croco_.nb_iterations_max, false);
            // TODO: are get_k()[0] and get_us()[0] the same?
            // Vector7d tau_ff = croco_reaching_.ddp_->get_k()[0];
            Vector7d tau_ff = croco_reaching_.ddp_->get_us()[0];
            //////////////////////////////////////

            // Fill and send control message
            lfc_msgs::Eigen::Control ctrl_eig;
            ctrl_eig.initial_state.joint_state.position = q;
            ctrl_eig.initial_state.joint_state.velocity = v;
            ctrl_eig.feedforward = tau_ff;
            ctrl_eig.feedback_gain = croco_reaching_.ddp_->get_K()[0];
            lfc_msgs::Control ctrl_msg;
            lfc_msgs::controlEigenToMsg(ctrl_eig, ctrl_msg);
            control_pub_.publish(ctrl_msg);
        }

        // sensor callback
        bool first_sensor_msg_received_;
        realtime_tools::RealtimeBox<Vector7d> q_init_rtbox_;
        realtime_tools::RealtimeBox<Eigen::Matrix<double, 14, 1>> current_x_rtbox_;
        pin::SE3 T_b_e0_;

        // pose ref callback
        bool first_pose_ref_msg_received_;
        pin::SE3 T_w_t0_;
        realtime_tools::RealtimeBox<pin::SE3> T_be_rtbox_;

        // Solve state machine
        bool first_solve_;

        // Pinocchio objects
        pin::Model model_pin_;
        pin::Data data_pin_;
        std::string ee_frame_pin_;
        pin::FrameIndex ee_frame_id_;

        // MPC formulation
        CrocoddylReaching croco_reaching_;
        CrocoddylConfig config_croco_;

        // Publisher of commands
        ros::Publisher control_pub_;

        // Subscriber to robot sensor from linearized ctrl
        ros::Subscriber sensor_sub_;

        // Subscriber to pose reference topic
        ros::Subscriber pose_ref_sub_;
    };

} // namespace panda_torque_mpc

int main(int argc, char **argv)
{

    ros::init(argc, argv, "croccodyl_motion_server_node");
    ros::NodeHandle nh;
    std::string robot_sensors_topic_sub = "robot_sensors";
    std::string motion_server_control_topic_pub = "motion_server_control";
    std::string ee_pose_ref_topic_sub = "ee_pose_ref";
    auto motion_server = panda_torque_mpc::CrocoMotionServer(nh, robot_sensors_topic_sub, motion_server_control_topic_pub, ee_pose_ref_topic_sub);

    int freq_node = (int) 1.0/motion_server.config_croco_.dt_ocp;
    ros::Rate loop_rate(freq_node);

    while (ros::ok())
    {
        motion_server.solve_and_send();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 1;
}