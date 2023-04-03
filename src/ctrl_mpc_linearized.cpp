#include "panda_torque_mpc/ctrl_mpc_linearized.h"

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka/robot_state.h>

#include <linear_feedback_controller_msgs/eigen_conversions.hpp>

namespace panda_torque_mpc
{

    bool CtrlMpcLinearized::init(hardware_interface::RobotHW *robot_hw,
                                 ros::NodeHandle &nh)
    {
        ///////////////////
        // Load parameters
        ///////////////////
        std::string arm_id;
        if (!get_param_error_tpl<std::string>(nh, arm_id, "arm_id"))
            return false;

        if (!get_param_error_tpl<double>(nh, Kp_jsid_, "Kp_jsid"))
                    return false;
        if (!get_param_error_tpl<double>(nh, Kd_jsid_, "Kd_jsid"))
                    return false;

        // Panda
        std::vector<std::string> joint_names;
        if (!get_param_error_tpl<std::vector<std::string>>(nh, joint_names, "joint_names",
                                                           [](std::vector<std::string> v)
                                                           { return v.size() == 7; }))
            return false;

        double publish_log_rate(30.0);
        if (!nh.getParam("publish_log_rate", publish_log_rate))
        {
            ROS_INFO_STREAM("CtrlMpcLinearized: publish_log_rate not found. Defaulting to " << publish_log_rate);
        }
        rate_trigger_logs_ = franka_hw::TriggerRate(publish_log_rate);

        if (!get_param_error_tpl<double>(nh, alpha_dq_filter_, "alpha_dq_filter"))
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

        ///////////////////
        // Claim interfaces
        ///////////////////
        // Retrieve resource FrankaStateHandle
        auto *franka_state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
        if (franka_state_interface == nullptr)
        {
            ROS_ERROR("CtrlMpcLinearized: Could not get Franka state interface from hardware");
            return false;
        }
        try
        {
            franka_state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(franka_state_interface->getHandle(arm_id + "_robot"));
        }
        catch (const hardware_interface::HardwareInterfaceException &e)
        {
            ROS_ERROR_STREAM("CtrlMpcLinearized: Exception getting franka state handle: " << e.what());
            return false;
        }

        // Retrieve resource FrankaModelHandle
        auto *model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
        if (model_interface == nullptr)
        {
            ROS_ERROR_STREAM("CtrlMpcLinearized: Error getting model interface from hardware");
            return false;
        }
        try
        {
            franka_model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(model_interface->getHandle(arm_id + "_model"));
        }
        catch (hardware_interface::HardwareInterfaceException &e)
        {
            ROS_ERROR_STREAM("CtrlMpcLinearized: Exception getting model handle from interface: " << e.what());
            return false;
        }

        // Retrieve resource FrankaModelHandle
        auto *effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
        if (effort_joint_interface == nullptr)
        {
            ROS_ERROR_STREAM("CtrlMpcLinearized: Error getting effort joint interface from hardware");
            return false;
        }
        for (size_t i = 0; i < 7; ++i)
        {
            try
            {
                joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
            }
            catch (const hardware_interface::HardwareInterfaceException &e)
            {
                ROS_ERROR_STREAM("CtrlMpcLinearized: Exception getting joint handles: " << e.what());
                return false;
            }
        }

        // Logs publishers
        task_pose_publisher_.init(nh, "task_pose_comparison", 1);
        task_twist_publisher_.init(nh, "task_twist_comparison", 1);
        torques_publisher_.init(nh, "joint_torques_comparison", 1);

        std::string motion_server_sub_topic = "motion_server_sub";
        ros::Subscriber motion_server_control_topic_sub = nh.subscribe(motion_server_sub_topic, 1, &CtrlMpcLinearized::callback_motion_server, this);


        // init some variables
        dq_filtered_ = Vector7d::Zero();
        
        // Controller state machine
        bool control_ref_from_ddp_node_received_ = false;
        // ros::Time t0_mpc_first_msg_ = ros;
        // double  dt_transition_jsid_to_mpc_ = 1000; 

        return true;
    }

    void CtrlMpcLinearized::starting(const ros::Time &t0)
    {
        ROS_INFO_STREAM("CtrlMpcLinearized::starting");
        t_init_ = t0;
        q_init_ = Eigen::Map<const Vector7d>(franka_state_handle_->getRobotState().q.data());
        pin::forwardKinematics(model_pin_, data_pin_, q_init_);
        pin::updateFramePlacements(model_pin_, data_pin_);
        T_b_e0_ = data_pin_.oMf[ee_frame_id_];

        ROS_INFO_STREAM("CtrlMpcLinearized::starting T_b_e0_: \n" << T_b_e0_);

    }

    void CtrlMpcLinearized::update(const ros::Time &t, const ros::Duration &period)
    {
        TicTac tictac;

        // Time since start of the controller
        double Dt = (t - t_init_).toSec();

        // Retrieve current measured robot state
        franka::RobotState robot_state = franka_state_handle_->getRobotState(); // return a const& of RobotState object -> not going to be modified
        Eigen::Map<Vector7d> q_m(robot_state.q.data());
        Eigen::Map<Vector7d> dq_m(robot_state.dq.data());
        Eigen::Map<Vector7d> tau_m(robot_state.tau_J.data()); // measured torques -> naturally contains gravity torque
        // End effector computed state
        // FK and differential FK
        pin::forwardKinematics(model_pin_, data_pin_, q_m, dq_m);
        pin::updateFramePlacements(model_pin_, data_pin_);
        pin::SE3 T_o_e_m = data_pin_.oMf[ee_frame_id_];
        pin::Motion nu_o_e_m = pin::getFrameVelocity(model_pin_, data_pin_, ee_frame_id_, pin::LOCAL_WORLD_ALIGNED);

        // filter the joint velocity measurements
        dq_filtered_ = (1 - alpha_dq_filter_) * dq_filtered_ + alpha_dq_filter_ * dq_m;


        /////////////////////////////////////////////////////////////
        /////////////////////////////////////////////////////////////
        //////////////////// Compute desired torque /////////////////
        /////////////////////////////////////////////////////////////
        /////////////////////////////////////////////////////////////
        TicTac tictac_comp;

        /**
         * State machine start of controller
         *
         * if (!control_ref_from_ddp_node_received)
         *     torques = PD(q0)
         * else if ( t_since_first_ddp_ctrl_reveived < t_transition)
         *     torque_PD = PD(q0)
         *     torque_mpc = linear_fbk(u_ddp, x_ddp K, x_m)
         *     alpha = t_since_first_ddp_ctrl_reveived/t_transition
         *     torques = (1 - alpha) * torque_PD + alpha * torque_mpc
         * else
         *     torque_mpc = linear_fbk(u_ddp, x_ddp K, x_m)
         *
         */


        Vector7d tau_d;
        if (control_ref_from_ddp_node_received_)
        {
            Vector7d dq_ref = Vector7d::Zero();
            tau_d = compute_torque_jsid(q_m, dq_m, q_init_, dq_ref);
        }
        else if ((t - t0_mpc_first_msg_).toSec() < dt_transition_jsid_to_mpc_)
        {
            Vector7d dq_ref = Vector7d::Zero();
            Vector7d tau_jsid = compute_torque_jsid(q_m, dq_m, q_init_, dq_ref);
            Vector7d tau_linear_mpc = compute_torque_mpc_linear_feedback(q_m, dq_m, u0_mpc_, x0_mpc_, K_ricatti_);
            double alpha_tau = (t - t0_mpc_first_msg_).toSec() / dt_transition_jsid_to_mpc_;
            // alpha = 0 -> pure jsid, alpha = 1 pure mpc
            tau_d = alpha_tau * tau_linear_mpc + (1 - alpha_tau) * tau_jsid;
        }
        else
        {
            tau_d = compute_torque_mpc_linear_feedback(q_m, dq_m, u0_mpc_, x0_mpc_, K_ricatti_);
        }

        // Remove gravity to send the torques to the robot
        tau_d -= pin::computeGeneralizedGravity(model_pin_, data_pin_, q_m);
        
        tictac_comp.print_tac("compute_desired_torque took (ms): ");


        /////////////////////////////////////////////////////////////
        // Publish robot state 
        publish_robot_state(q_m, dq_m);
        /////////////////////////////////////////////////////////////



        /////////////////////////////////////////////////////////////
        /////////////////////////////////////////////////////////////
        /////////////////////////////////////////////////////////////
        /////////////////////////////////////////////////////////////

        // Maximum torque difference with a sampling rate of 1 kHz. The maximum torque rate is
        // 1000 * (1 / sampling_time).
        // Vector7d tau_d_saturated = saturateTorqueRate(tau_d, Eigen::Map<Vector7d>(robot_state.tau_J_d.data()));
        Vector7d tau_d_saturated = tau_d;

        // Send Torque Command
        for (size_t i = 0; i < 7; ++i)
        {
            joint_handles_[i].setCommand(tau_d_saturated[i]);
        }

        // ///////////////////
        // // Publish logs
        // // Takes about 5 us -> negligeable
        // if (rate_trigger_() && torques_publisher_.trylock())
        // {
        //     // TicTac tt_publish;
        //     // Refactor to: publish_logs(publishers, last_vals, measured_vals) ?

        //     /**
        //      * Measured torque in simulation returns -tau while running with real robot returns tau --___--
        //      */
        //     tau_m = -tau_m; // SIMULATION
        //     // tau_m = tau_m;  // REAL

        //     // torque
        //     Vector7d tau_error = last_tau_d_ - tau_m;
        //     // EE pose
        //     Eigen::Vector3d p_o_e_err = T_o_e_m.translation() - last_x_r_.translation();
        //     Eigen::Quaterniond quat_r(last_x_r_.rotation());
        //     Eigen::Quaterniond quat_m(T_o_e_m.rotation());
        //     Eigen::Quaterniond quat_err = quat_r.inverse() * quat_m;
        //     // EE twist
        //     Eigen::Vector3d v_o_e_err = nu_o_e_m.linear() - dx_r.linear();
        //     Eigen::Vector3d omg_o_e_err = nu_o_e_m.angular() - dx_r.angular();

        //     // EE Twists linear part
        //     task_twist_publisher_.msg_.commanded.linear.x = dx_r.linear()[0];
        //     task_twist_publisher_.msg_.commanded.linear.y = dx_r.linear()[1];
        //     task_twist_publisher_.msg_.commanded.linear.z = dx_r.linear()[2];
        //     task_twist_publisher_.msg_.measured.linear.x = nu_o_e_m.linear()[0];
        //     task_twist_publisher_.msg_.measured.linear.y = nu_o_e_m.linear()[1];
        //     task_twist_publisher_.msg_.measured.linear.z = nu_o_e_m.linear()[2];
        //     task_twist_publisher_.msg_.error.linear.x = v_o_e_err[0];
        //     task_twist_publisher_.msg_.error.linear.y = v_o_e_err[1];
        //     task_twist_publisher_.msg_.error.linear.z = v_o_e_err[2];

        //     // EE Twists angular part
        //     task_twist_publisher_.msg_.commanded.angular.x = dx_r.angular()[0];
        //     task_twist_publisher_.msg_.commanded.angular.y = dx_r.angular()[1];
        //     task_twist_publisher_.msg_.commanded.angular.z = dx_r.angular()[2];
        //     task_twist_publisher_.msg_.measured.angular.x = nu_o_e_m.angular()[0];
        //     task_twist_publisher_.msg_.measured.angular.y = nu_o_e_m.angular()[1];
        //     task_twist_publisher_.msg_.measured.angular.z = nu_o_e_m.angular()[2];
        //     task_twist_publisher_.msg_.error.angular.x = omg_o_e_err[0];
        //     task_twist_publisher_.msg_.error.angular.y = omg_o_e_err[1];
        //     task_twist_publisher_.msg_.error.angular.z = omg_o_e_err[2];

        //     // End effector position
        //     task_pose_publisher_.msg_.commanded.position.x = x_r.translation()[0];
        //     task_pose_publisher_.msg_.commanded.position.y = x_r.translation()[1];
        //     task_pose_publisher_.msg_.commanded.position.z = x_r.translation()[2];
        //     task_pose_publisher_.msg_.measured.position.x = T_o_e_m.translation()[0];
        //     task_pose_publisher_.msg_.measured.position.y = T_o_e_m.translation()[1];
        //     task_pose_publisher_.msg_.measured.position.z = T_o_e_m.translation()[2];
        //     task_pose_publisher_.msg_.error.position.x = p_o_e_err[0];
        //     task_pose_publisher_.msg_.error.position.y = p_o_e_err[1];
        //     task_pose_publisher_.msg_.error.position.z = p_o_e_err[2];

        //     // Quaternion
        //     task_pose_publisher_.msg_.commanded.orientation.x = quat_r.x();
        //     task_pose_publisher_.msg_.commanded.orientation.y = quat_r.y();
        //     task_pose_publisher_.msg_.commanded.orientation.z = quat_r.z();
        //     task_pose_publisher_.msg_.commanded.orientation.w = quat_r.w();
        //     task_pose_publisher_.msg_.measured.orientation.x = quat_m.x();
        //     task_pose_publisher_.msg_.measured.orientation.y = quat_m.y();
        //     task_pose_publisher_.msg_.measured.orientation.z = quat_m.z();
        //     task_pose_publisher_.msg_.measured.orientation.w = quat_m.w();
        //     task_pose_publisher_.msg_.error.orientation.x = quat_err.x();
        //     task_pose_publisher_.msg_.error.orientation.y = quat_err.y();
        //     task_pose_publisher_.msg_.error.orientation.z = quat_err.z();
        //     task_pose_publisher_.msg_.error.orientation.w = quat_err.w();

        //     // Size 7 vectors
        //     for (size_t i = 0; i < 7; ++i)
        //     {
        //         torques_publisher_.msg_.commanded[i] = last_tau_d_[i];
        //         torques_publisher_.msg_.measured[i] = tau_m[i];
        //         torques_publisher_.msg_.error[i] = tau_error[i];
        //     }

        //     task_pose_publisher_.unlockAndPublish();
        //     task_twist_publisher_.unlockAndPublish();
        //     torques_publisher_.unlockAndPublish();

        //     // std::cout << std::setprecision(9) << "publish took (ms): " << tt_publish.tac() << std::endl;
        // }

        // Store previous desired/reference values
        last_tau_d_ = tau_d_saturated + Eigen::Map<Vector7d>(franka_model_handle_->getGravity().data());

        tictac.print_tac("update() took (ms): ");
    }

    Vector7d CtrlMpcLinearized::compute_torque_jsid(const Vector7d &q_m, const Vector7d &dq_m, const Vector7d &q_ref, const Vector7d &dq_ref)
    {
        Vector7d ddq_d = - Kp_jsid_ * (q_m - q_ref) - Kd_jsid_ * (dq_m - dq_ref);
        pin::rnea(model_pin_, data_pin_, q_m, dq_m, ddq_d);
        return data_pin_.tau;
    }

    Vector7d CtrlMpcLinearized::compute_torque_mpc_linear_feedback(const Vector7d &q_m, 
                                                                   const Vector7d &dq_m, 
                                                                   const Eigen::Matrix<double, 7, 1> &u0_mpc, 
                                                                   const Eigen::Matrix<double, 14, 1> &x0_mpc, 
                                                                   const Eigen::Matrix<double, 7, 14> &K_ricatti)
    {
        Eigen::Matrix<double, 14, 1> x_m; 
        x_m << q_m, dq_m;

        // Vector7d tau_d = u0_mpc_;

        Vector7d tau_d = u0_mpc + K_ricatti * (x0_mpc - x_m);

        return tau_d;
    }


    void CtrlMpcLinearized::callback_motion_server(const linear_feedback_controller_msgs::Control& ctrl_msg)
    {
        linear_feedback_controller_msgs::Eigen::Control ctrl_eig;
        linear_feedback_controller_msgs::controlMsgToEigen(ctrl_msg, ctrl_eig);
        u0_mpc_ = ctrl_eig.feedforward;
        x0_mpc_ << ctrl_eig.initial_state.joint_state.position, ctrl_eig.initial_state.joint_state.velocity;
        K_ricatti_ = ctrl_eig.feedback_gain;
    }


    void CtrlMpcLinearized::publish_robot_state(const Eigen::VectorXd &q_m, const Eigen::VectorXd &dq_m)
    {
        linear_feedback_controller_msgs::Sensor robot_sensor_msg;
        linear_feedback_controller_msgs::Eigen::Sensor robot_sensor_eig;
        robot_sensor_eig.joint_state.position = q_m;
        robot_sensor_eig.joint_state.velocity = dq_m;
        linear_feedback_controller_msgs::sensorEigenToMsg(robot_sensor_eig, robot_sensor_msg);

        if (robot_state_publisher_.trylock())
        {
            robot_state_publisher_.msg_ = robot_sensor_msg;
            robot_state_publisher_.unlockAndPublish();
        }
    }

    void CtrlMpcLinearized::stopping(const ros::Time &t0)
    {
        ROS_INFO_STREAM("CtrlMpcLinearized::stopping");
    }

} // namespace panda_torque_mpc

PLUGINLIB_EXPORT_CLASS(panda_torque_mpc::CtrlMpcLinearized,
                       controller_interface::ControllerBase)
