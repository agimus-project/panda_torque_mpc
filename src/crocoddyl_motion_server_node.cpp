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
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "panda_torque_mpc/common.h"
#include "panda_torque_mpc/crocoddyl_reaching.h"

#include "geometry_msgs/PoseStamped.h"



namespace panda_torque_mpc
{
    namespace lfc_msgs = linear_feedback_controller_msgs;

    class CrocoMotionServer
    {
    public:
        CrocoMotionServer(ros::NodeHandle &nh,
                          std::string robot_sensors_topic_sub,
                          std::string control_topic_pub,
                          std::string ee_pose_ref_t265_topic_sub,
                          std::string cam_pose_ref_viz_topic_pub
                          )
        {
            bool params_success = true;
            ///////////////////
            // Load parameters
            ///////////////////
            std::string arm_id;
            params_success = get_param_error_tpl<std::string>(nh, arm_id, "arm_id") && params_success;

            // Croco params
            int nb_shooting_nodes, nb_iterations_max;
            double dt_ocp, w_frame_running, w_frame_terminal, w_x_reg_running, w_x_reg_terminal, scale_q_vs_v_reg, w_u_reg_running;
            std::vector<double> armature, diag_u_reg_running;
            std::vector<double> pose_e_c, pose_c_o_ref;  // px,py,pz, qx,qy,qz,qw
            bool reference_is_placement;

            params_success = get_param_error_tpl<int>(nh, nb_shooting_nodes, "nb_shooting_nodes") && params_success;
            params_success = get_param_error_tpl<double>(nh, dt_ocp, "dt_ocp") && params_success;
            params_success = get_param_error_tpl<int>(nh, nb_iterations_max, "nb_iterations_max") && params_success;
            params_success = get_param_error_tpl<bool>(nh, reference_is_placement, "reference_is_placement") && params_success;
            params_success = get_param_error_tpl<bool>(nh, keep_original_ee_rotation_, "keep_original_ee_rotation") && params_success;
            params_success = get_param_error_tpl<double>(nh, w_frame_running, "w_frame_running") && params_success;
            params_success = get_param_error_tpl<double>(nh, w_frame_terminal, "w_frame_terminal") && params_success;
            params_success = get_param_error_tpl<double>(nh, w_x_reg_running, "w_x_reg_running") && params_success;
            params_success = get_param_error_tpl<double>(nh, w_x_reg_terminal, "w_x_reg_terminal") && params_success;
            params_success = get_param_error_tpl<double>(nh, scale_q_vs_v_reg, "scale_q_vs_v_reg") && params_success;
            params_success = get_param_error_tpl<double>(nh, w_u_reg_running, "w_u_reg_running") && params_success;

            params_success = get_param_error_tpl<double>(nh, w_u_reg_running, "w_u_reg_running") && params_success;


            params_success = get_param_error_tpl<std::vector<double>>(nh, armature, "armature",
                                                                      [](std::vector<double> v)
                                                                      { return v.size() == 7; }) && params_success;
            params_success = get_param_error_tpl<std::vector<double>>(nh, diag_u_reg_running, "diag_u_reg_running",
                                                                      [](std::vector<double> v)
                                                                      { return v.size() == 7; }) && params_success;

            params_success = get_param_error_tpl<std::vector<double>>(nh, pose_e_c, "pose_e_c",
                                                                      [](std::vector<double> v)
                                                                      { return v.size() == 7; }) && params_success;
            params_success = get_param_error_tpl<std::vector<double>>(nh, pose_c_o_ref, "pose_c_o_ref",
                                                                      [](std::vector<double> v)
                                                                      { return v.size() == 7; }) && params_success;




            // Load panda model with pinocchio
            std::string urdf_path;
            urdf_path = "/home/imitlearn/sanbox_mfourmy/ws_panda_ctrl/src/panda_torque_mpc/res/panda_inertias.urdf";
            // params_success = get_param_error_tpl<std::string>(nh, urdf_path, "urdf_path") && params_success;
            params_success = get_param_error_tpl<std::string>(nh, ee_frame_name_, "ee_frame_name") && params_success;

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
            ee_frame_id_ = model_pin_.getFrameId(ee_frame_name_);

            /////////////////////////////////////////////////
            //                MPC CONFIG                   //
            /////////////////////////////////////////////////
            config_croco_.T = nb_shooting_nodes;
            config_croco_.dt_ocp = dt_ocp;
            config_croco_.nb_iterations_max = nb_iterations_max;
            config_croco_.ee_frame_name = ee_frame_name_;
            config_croco_.reference_is_placement = reference_is_placement;
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
            control_pub_ = nh.advertise<lfc_msgs::Control>(control_topic_pub, 1);
            pose_ref_viz_pub_ = nh.advertise<geometry_msgs::PoseStamped>(cam_pose_ref_viz_topic_pub, 1);
            sensor_sub_ = nh.subscribe(robot_sensors_topic_sub, 10, &CrocoMotionServer::callback_sensor, this);
            pose_ref_t265_sub_ = nh.subscribe(ee_pose_ref_t265_topic_sub, 10, &CrocoMotionServer::callback_pose_ref_t265, this);
            std::string ee_pose_ref_visual_servoing_topic_sub = "ee_pose_ref_visual_servoing"; 
            pose_ref_tracker_sub_ = nh.subscribe(ee_pose_ref_visual_servoing_topic_sub, 10, &CrocoMotionServer::callback_pose_camera_object, this);

            // tf2
            // tf_listener_ = tf2_ros::TransformListener(tf_buffer_);

            // Init some variables
            first_camera_object_pose_received_ = false;
            first_pose_ref_msg_received_ = false;
            first_solve_ = true;

            last_pose_ref_ts_ = ros::Time::now();
            
            T_e_c_ = XYZQUATToSE3(pose_e_c);
            T_c_e_ = T_e_c_.inverse();
            T_c_o_ref_ = XYZQUATToSE3(pose_c_o_ref);
            T_o_c_ref_ = T_c_o_ref_.inverse();

        }

        void callback_pose_ref_t265(const geometry_msgs::PoseStamped &msg)
        {
            /**
             * If the first sensor state of the robot has not yet been received, no need to process the pose ref
             */
            last_pose_ref_ts_ = ros::Time::now();

            if (!first_camera_object_pose_received_)
            {
                return;
            }

            std::cout << "callback_pose_ref_t265 " << std::endl;

            pin::SE3 T_w_t = posemsg2SE3(msg.pose);

            // Cf ctrl_task_space_ID for instance for why this choice
            // TRANSLATION ref
            pin::SE3 T_e0_e = pin::SE3::Identity();
            auto R_e0_b = T_b_e0_.rotation().transpose();
            T_e0_e.translation() = R_e0_b * (T_w_t.translation() - T_w_t0_.translation());

            // // ROTATION ref  --> NOPE
            // Eigen::Matrix3d R_w_t0 = T_w_t0_.rotation();
            // Eigen::Matrix3d R_e0_t0 = R_e0_b* R_w_t0;
            // Eigen::Matrix3d R_t0_t = R_w_t0.transpose() * T_w_t.rotation();
            // T_e0_e.rotation() = R_e0_t0 * R_t0_t;

            // std::cout << "\n\nYOOOOOOOO" << std::endl;
            // std::cout << "R_w_t0\n" << R_w_t0 << std::endl;
            // std::cout << "R_e0_b\n" << R_e0_b << std::endl;
            // std::cout << "T_w_t\n" << T_w_t << std::endl;
            // std::cout << "R_t0_t\n" << R_t0_t << std::endl;
            // std::cout << "T_e0_e.rotation()\n" << T_e0_e.rotation() << std::endl;

            // Set reference pose
            // compose initial pose with relative/local transform
            pin::SE3 T_b_e_ref = T_b_e0_ * T_e0_e;

            // RT safe setting
            T_b_e_ref_rtbox_.set(T_b_e_ref);


            if (!first_pose_ref_msg_received_)
            {
                T_w_t0_ = T_w_t;
                first_pose_ref_msg_received_ = true;
            }

        }


        void callback_pose_camera_object(const geometry_msgs::PoseStamped &msg_pose_c_o)
        {
            /**
             * If the first sensor state of the robot has not yet been received, no need to process the pose ref
             */
            last_pose_ref_ts_ = ros::Time::now();

            if (!first_camera_object_pose_received_)
            {
                return;
            }

            std::cout << "callback_pose_camera_object " << std::endl;

            pin::SE3 T_c_o_meas = posemsg2SE3(msg_pose_c_o.pose);
            pin::SE3 T_o_c_meas = T_c_o_meas.inverse();

            std::cout << "\n\n\n\n\ncallback_pose_camera_object" << std::endl;
            std::cout << "T_c_o_meas\n" << T_c_o_meas << std::endl;
            std::cout << "T_c_o_ref_\n" << T_c_o_ref_ << std::endl;

            pin::SE3 T_b_e;
            T_b_e_rtbox_.get(T_b_e);

            pin::SE3 T_b_c_ref = T_b_e * T_e_c_ * T_c_o_meas * T_o_c_ref_;  // ORIGINAL
            // pin::SE3 T_b_c_ref = T_b_e * T_e_c_ * T_c_o_ref_ * T_o_c_meas;  // NOPPPPPPPPE
            pin::SE3 T_b_e_ref = T_b_c_ref * T_c_e_;

            // !!!!!!!!!!!!!
            // !!!!!!!!!!!!!
            // Test without also
            if (keep_original_ee_rotation_)
            {
                T_b_e_ref.rotation() = T_b_e0_.rotation();
            }
            // !!!!!!!!!!!!!
            // !!!!!!!!!!!!!
            // !!!!!!!!!!!!!


            std::cout << "T_b_e0_" << std::endl;
            std::cout << T_b_e0_ << std::endl;
            std::cout << "T_b_e" << std::endl;
            std::cout << T_b_e << std::endl;
            std::cout << "T_b_e_ref" << std::endl;
            std::cout << T_b_e_ref << std::endl;

            // RT safe setting
            T_b_e_ref_rtbox_.set(T_b_e_ref);


            if (!first_pose_ref_msg_received_)
            {
                first_pose_ref_msg_received_ = true;
            }

            // Send message with reference camera pose
            pin::SE3 T_c_cref = T_c_o_meas * T_o_c_ref_;
            geometry_msgs::PoseStamped msg_pose_ccref;
            msg_pose_ccref.pose = SE32posemsg(T_c_cref);
            msg_pose_ccref.header = msg_pose_c_o.header;
            pose_ref_viz_pub_.publish(msg_pose_ccref);
        }

        void callback_sensor(const lfc_msgs::Sensor &sensor_msg)
        {
            // Recover latest robot state from the sensor msg
            t_sensor_ = sensor_msg.header.stamp;
            lfc_msgs::Eigen::Sensor sensor_eig;
            lfc_msgs::sensorMsgToEigen(sensor_msg, sensor_eig);
            Eigen::Matrix<double, 14, 1> current_x;
            current_x << sensor_eig.joint_state.position, sensor_eig.joint_state.velocity;
            current_x_rtbox_.set(current_x);

            pin::forwardKinematics(model_pin_, data_pin_, sensor_eig.joint_state.position);
            pin::updateFramePlacements(model_pin_, data_pin_);

            pin::SE3 T_b_e = data_pin_.oMf[ee_frame_id_];
            T_b_e_rtbox_.set(T_b_e);

            // Separate callback for reference?
            if (!first_camera_object_pose_received_)
            {
                T_b_e0_ = T_b_e;

                q_init_rtbox_.set(sensor_eig.joint_state.position);
                first_camera_object_pose_received_ = true;
            }
        }

        void solve_and_send()
        {
            // Do nothing if no pose reference and sensor state has been received
            if (!(first_camera_object_pose_received_ && first_pose_ref_msg_received_))
            {
                return;
            }

            // Retrieve reference in thread-safe way
            pin::SE3 T_b_e_ref;
            T_b_e_ref_rtbox_.get(T_b_e_ref);

            // Solve and send
            std::cout << "\n -------- T_b_e_ref\n" << T_b_e_ref << std::endl;

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

                /**
                 * 
                 * Shift trajectory by 1 node <==> config_croco_.dt_ocp
                 * 
                 * !!! HYP: config_croco_.dt_ocp == freq_node
                 * xs_init.insert(std::begin(xs_init), current_x);
                */
                xs_init.insert(std::begin(xs_init), xs_init[0]);
                xs_init.erase(std::end(xs_init) - 1);
                us_init.insert(std::begin(us_init), us_init[0]);
                us_init.erase(std::end(us_init) - 1);
            }

            // Set initial state and end-effector ref
            croco_reaching_.ddp_->get_problem()->set_x0(current_x);

            // bool reaching_task_is_active = (ros::Time::now() - last_pose_ref_ts_).toSec() < 0.5;
            // Deactivating reaching task would requires to re-equilibrate the OCP weights, easier to keep the last reference active
            bool reaching_task_is_active = true;
            if (config_croco_.reference_is_placement)
            {
                std::cout << "config_croco_.reference_is_placement\n" << T_b_e_ref << std::endl;
                std::cout << "T_b_e_ref\n" << T_b_e_ref << std::endl;
                std::cout << "T_b_e0_\n" << T_b_e0_ << std::endl;
                croco_reaching_.set_ee_ref_placement(T_b_e_ref, reaching_task_is_active);
            }
            else
            {
                std::cout << "config_croco_.reference_is_NOOOOOOT_placement" << std::endl;
                std::cout << "T_b_e_ref\n" << T_b_e_ref << std::endl;
                std::cout << "T_b_e0_\n" << T_b_e0_ << std::endl;
                croco_reaching_.set_ee_ref_translation(T_b_e_ref.translation(), reaching_task_is_active);
            }
            croco_reaching_.set_posture_ref(x_init);

            TicTac tt_solve;
            bool success_solve = croco_reaching_.solve(xs_init, us_init);
            tt_solve.print_tac("");
            // if problem not ready or no good solution, don't send a solution
            if (!success_solve) return;
            //////////////////////////////////////

            // Fill and send control message
            lfc_msgs::Eigen::Control ctrl_eig;
            ctrl_eig.initial_state.joint_state.position = q;
            ctrl_eig.initial_state.joint_state.velocity = v;
            ctrl_eig.feedforward = croco_reaching_.get_tau_ff();
            ctrl_eig.feedback_gain = croco_reaching_.get_ricatti_mat();
            lfc_msgs::Control ctrl_msg;
            lfc_msgs::controlEigenToMsg(ctrl_eig, ctrl_msg);
            control_pub_.publish(ctrl_msg);
        }

        // sensor callback
        bool first_camera_object_pose_received_;
        ros::Time t_sensor_;
        realtime_tools::RealtimeBox<Vector7d> q_init_rtbox_;
        realtime_tools::RealtimeBox<Eigen::Matrix<double, 14, 1>> current_x_rtbox_;
        pin::SE3 T_b_e0_;
        realtime_tools::RealtimeBox<pin::SE3> T_b_e_rtbox_;

        // Visual servoing
        // Camera calibration
        pin::SE3 T_e_c_;
        pin::SE3 T_c_e_;
        // Pose reference
        pin::SE3 T_o_c_ref_;
        pin::SE3 T_c_o_ref_;


        // pose ref callback
        bool first_pose_ref_msg_received_;
        ros::Time last_pose_ref_ts_;
        pin::SE3 T_w_t0_;
        realtime_tools::RealtimeBox<pin::SE3> T_b_e_ref_rtbox_;

        // Solve state machine
        bool first_solve_;


        // Pinocchio objects
        pin::Model model_pin_;
        pin::Data data_pin_;
        std::string ee_frame_name_;
        pin::FrameIndex ee_frame_id_;

        // MPC formulation
        CrocoddylReaching croco_reaching_;
        CrocoddylConfig config_croco_;

        // Publisher of commands
        ros::Publisher control_pub_;
        ros::Publisher pose_ref_viz_pub_;

        // Subscriber to robot sensor from linearized ctrl
        ros::Subscriber sensor_sub_;

        // Subscriber to pose reference topic
        ros::Subscriber pose_ref_t265_sub_;
        ros::Subscriber pose_ref_tracker_sub_;

        // tf2
        tf2_ros::Buffer tf_buffer_;
        // tf2_ros::TransformListener tf_listener_;

        // tf2 frame ids
        // T265 demo
        std::string world_frame_ = "camera_odom_frame";  // static inertial "world=w" frame    
        std::string camera_pose_frame_ = "camera_pose_frame";  // moving "camera=c" frame
        // camera2object from icg (eye in hand case)
        std::string camera_color_optical_frame_ = "camera_color_optical_frame";
        std::string object_frame_ = "object_frame";

        // DEMO mode
        bool keep_original_ee_rotation_ = false;

};

} // namespace panda_torque_mpc

int main(int argc, char **argv)
{

    ros::init(argc, argv, "crocoddyl_motion_server_node");
    ros::NodeHandle nh;
    std::string robot_sensors_topic_sub = "robot_sensors";
    std::string control_topic_pub = "motion_server_control";
    std::string ee_pose_ref_t265_topic_sub = "ee_pose_ref";
    std::string cam_pose_ref_viz_topic_pub = "cam_pose_ref_viz";
    auto motion_server = panda_torque_mpc::CrocoMotionServer(
                            nh, 
                            robot_sensors_topic_sub,
                            control_topic_pub,
                            ee_pose_ref_t265_topic_sub,
                            cam_pose_ref_viz_topic_pub
                            );

    int freq_node = (int) 1.0/motion_server.config_croco_.dt_ocp;
    ros::Rate loop_rate(freq_node);

    // // Listener fills the tf2 buffer
    // tf2_ros::TransformListener tf_listener(motion_server.tf_buffer_);

    // double dt_to_fill_the_buffer = 1.0;
    // ros::Rate rate_sleep_init(dt_to_fill_the_buffer);
    // std::cout << "Filling tf transform buffer for (sec)" << dt_to_fill_the_buffer << std::endl;
    // rate_sleep_init.sleep();


    while (ros::ok())
    {
        // motion_server.retrieve_pose_ref_from_tf();
        motion_server.solve_and_send();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 1;
}