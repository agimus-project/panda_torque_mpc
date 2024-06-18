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
    namespace lfc_msgs = linear_feedback_controller_msgs;

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
        if (!get_param_error_tpl<double>(nh, dt_transition_jsid_to_mpc_, "dt_transition_jsid_to_mpc"))
            return false;
        if (!get_param_error_tpl<bool>(nh, use_riccati_gains_, "use_riccati_gains"))
                return false;

        // Panda
        std::vector<std::string> joint_names;
        if (!get_param_error_tpl<std::vector<std::string>>(nh, joint_names, "joint_names",
                                                           [](std::vector<std::string> v)
                                                           { return v.size() == 7; }))
            return false;

        double publish_log_rate(100.0);
        if (!nh.getParam("publish_log_rate", publish_log_rate))
        {
            ROS_INFO_STREAM("CtrlMpcLinearized: publish_log_rate not found. Defaulting to " << publish_log_rate);
        }
        rate_trigger_ = franka_hw::TriggerRate(publish_log_rate);

        if (!get_param_error_tpl<std::string>(nh, ee_frame_name_, "ee_frame_name")) return false;

        std::string robot_description;
        if(!get_param_error_tpl<std::string>(nh, robot_description, "/robot_description")) return false;

        model_pin_ = loadPandaPinocchio(robot_description);
        data_pin_ = pin::Data(model_pin_);

        std::cout << " ee_frame_name_ : " << ee_frame_name_ << std::endl;
        std::cout << " model_pin_.getFrameId(ee_frame_name_) : " << model_pin_.getFrameId(ee_frame_name_) << std::endl;
        std::cout <<  " model_pin_.getFrameId(do not exist) : " << model_pin_.getFrameId("do not exist") << std::endl;

        if ((model_pin_.nq != 7) || (model_pin_.name != "panda"))
        {
            ROS_ERROR_STREAM("Problem when loading the robot urdf");
            return false;
        }

        // Define corresponding frame id for pinocchio and Franka (see ctrl_model_pinocchio_vs_franka)
        ee_frame_id_ = model_pin_.getFrameId(ee_frame_name_);
        

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
        configurations_publisher_.init(nh, "joint_configurations_comparison", 10);
        velocities_publisher_.init(nh, "joint_velocities_comparison", 10);
        torques_publisher_.init(nh, "joint_torques_comparison", 10);

        
        // Robot sensor publisher 
        robot_state_publisher_.init(nh, "robot_sensors", 10);

        std::string motion_server_sub_topic = "motion_server_control";
        motion_server_control_topic_sub_ = nh.subscribe(motion_server_sub_topic, 10, &CtrlMpcLinearized::callback_motion_server, this);

        return true;
    }

    void CtrlMpcLinearized::starting(const ros::Time &t0)
    {
        ROS_INFO_STREAM("CtrlMpcLinearized::starting");
        t_init_ = t0;
        q_init_ = Eigen::Map<const Vector7d>(franka_state_handle_->getRobotState().q.data());
        
        // Controller state machine
        control_ref_from_ddp_node_received_ = false;
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
        // Retrieve mpc callback variables, whether or not rightfully initialized
        Eigen::Matrix<double, 14, 1> x0_mpc; 
        Eigen::Matrix<double, 7, 1> u0_mpc;
        Eigen::Matrix<double, 7, 14> K_ricatti;
        x0_mpc_rtbox_.get(x0_mpc);
        u0_mpc_rtbox_.get(u0_mpc);
        K_ricatti_rtbox_.get(K_ricatti);
        
        if (!control_ref_from_ddp_node_received_)
        {
            std::cout << "control_ref_from_ddp_node_received_ == false" << std::endl;
            Vector7d dq_ref = Vector7d::Zero();
            tau_d = compute_torque_jsid(q_m, dq_m, q_init_, dq_ref);
        }
        else if ((t - t0_mpc_first_msg_).toSec() < dt_transition_jsid_to_mpc_)
        {
            std::cout << "TRANSITION: " << (t - t0_mpc_first_msg_).toSec() << " < " << dt_transition_jsid_to_mpc_ << std::endl;
            Vector7d dq_ref = Vector7d::Zero();
            Vector7d tau_jsid = compute_torque_jsid(q_m, dq_m, q_init_, dq_ref);

            Vector7d tau_linear_mpc = compute_torque_mpc_linear_feedback(q_m, dq_m, u0_mpc, x0_mpc, K_ricatti);
            double alpha_tau = (t - t0_mpc_first_msg_).toSec() / dt_transition_jsid_to_mpc_;
            // alpha = 0 -> pure jsid, alpha = 1 pure mpc
            tau_d = alpha_tau * tau_linear_mpc + (1 - alpha_tau) * tau_jsid;
        }
        else
        {
            std::cout << "STEADY STATE" << std::endl;
            tau_d = compute_torque_mpc_linear_feedback(q_m, dq_m, u0_mpc, x0_mpc, K_ricatti);
        }

        // Remove gravity to send the torques to the robot
        tau_d -= pin::computeGeneralizedGravity(model_pin_, data_pin_, q_m);

        // std::cout << "Sent tau_d: " << tau_d.transpose() << std::endl;
        tictac_comp.print_tac("compute_desired_torque took (ms): ");
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


        /////////////////////////////////////////////////////////////
        // Publish robot state
        publish_robot_state(q_m, dq_m, t);
        /////////////////////////////////////////////////////////////

        ///////////////////
        // Publish logs
        // Takes about 5 us -> negligeable
        ///////////////////
        // Publish logs
        if (rate_trigger_() && torques_publisher_.trylock())
        {

            // Refactor to: publish_logs(publishers, last_vals, measured_vals) ?

            /**
             * Measured torque in simulation returns -tau while running with real robot returns tau --___--
             * Does NOT influence the control law though
             */
            tau_m = -tau_m; // SIMULATION
            // tau_m = tau_m;  // REAL

            Vector7d q_r = x0_mpc.block<7,1>(0,0);
            Vector7d dq_r = x0_mpc.block<7,1>(7,0);
            Vector7d q_error = q_r - q_m;
            Vector7d dq_error = dq_r - dq_m;
            Vector7d tau_error = last_tau_d_ - tau_m;

            for (size_t i = 0; i < 7; ++i)
            {
                // Joint config
                configurations_publisher_.msg_.commanded[i] = q_r(i);
                configurations_publisher_.msg_.measured[i] = q_m[i];
                configurations_publisher_.msg_.error[i] = q_error[i];

                // Joint velocities
                velocities_publisher_.msg_.commanded[i] = dq_r(i);
                velocities_publisher_.msg_.measured[i] = dq_m[i];
                velocities_publisher_.msg_.error[i] = dq_error[i];

                // Joint torque
                torques_publisher_.msg_.commanded[i] = u0_mpc[i];
                // torques_publisher_.msg_.commanded[i] = last_tau_d_[i];
                torques_publisher_.msg_.measured[i] = tau_m[i];
                torques_publisher_.msg_.error[i] = tau_error[i];
            }

            configurations_publisher_.unlockAndPublish();
            velocities_publisher_.unlockAndPublish();
            torques_publisher_.unlockAndPublish();
        }


        // Store previous desired/reference values
        last_tau_d_ = tau_d_saturated + Eigen::Map<Vector7d>(franka_model_handle_->getGravity().data());

        tictac.print_tac("update() took (ms): ");
    }

    Vector7d CtrlMpcLinearized::compute_torque_jsid(const Vector7d &q_m, const Vector7d &dq_m, const Vector7d &q_ref, const Vector7d &dq_ref)
    {
        Vector7d ddq_d = -Kp_jsid_ * (q_m - q_ref) - Kd_jsid_ * (dq_m - dq_ref);
        pin::rnea(model_pin_, data_pin_, q_m, dq_m, ddq_d);
        return data_pin_.tau;
    }

    Vector7d CtrlMpcLinearized::compute_torque_mpc_linear_feedback(const Vector7d &q_m,
                                                                   const Vector7d &dq_m,
                                                                   const Eigen::Matrix<double, 7, 1> &u0_mpc,
                                                                   const Eigen::Matrix<double, 14, 1> &x0_mpc,
                                                                   const Eigen::Matrix<double, 7, 14> &K_ricatti)
    {
        Eigen::Matrix<double, 14, 1> x_m; x_m << q_m, dq_m;

        Vector7d tau_d;
        if (use_riccati_gains_){
            tau_d = u0_mpc + K_ricatti * (x0_mpc - x_m);
        }
        else {
            tau_d = u0_mpc;
        }

        return tau_d;
    }

    void CtrlMpcLinearized::callback_motion_server(const lfc_msgs::Control &ctrl_msg)
    {   
        //////////////////////////
        // Segfault: ctrl_eig matrices have no default dimension -> lfc_msgs::matrixMsgToEigen
        // creates an assertion error on the dimesions
        // lfc_msgs::Eigen::Control ctrl_eig;
        // lfc_msgs::controlMsgToEigen(ctrl_msg, ctrl_eig);
        // u0_mpc_ = ctrl_eig.feedforward;
        // x0_mpc_ << ctrl_eig.initial_state.joint_state.position, ctrl_eig.initial_state.joint_state.velocity;
        //////////////////////////

        //////////////////////////
        // Manually
        Eigen::Matrix<double, 14, 1> x0_mpc; 
        Eigen::Matrix<double, 7, 1> u0_mpc;
        Eigen::Matrix<double, 7, 14> K_ricatti;
        lfc_msgs::matrixMsgToEigen(ctrl_msg.feedforward, u0_mpc);
        lfc_msgs::matrixMsgToEigen(ctrl_msg.feedback_gain, K_ricatti);
        lfc_msgs::Eigen::JointState js_eig;
        lfc_msgs::jointStateMsgToEigen(ctrl_msg.initial_state.joint_state, js_eig);
        x0_mpc << js_eig.position, js_eig.velocity;

        x0_mpc_rtbox_.set(x0_mpc);
        u0_mpc_rtbox_.set(u0_mpc);
        K_ricatti_rtbox_.set(K_ricatti);
        //////////////////////////

        if (!control_ref_from_ddp_node_received_)
        {
            t0_mpc_first_msg_ = ros::Time::now();
        }
        control_ref_from_ddp_node_received_ = true;
    }

    void CtrlMpcLinearized::publish_robot_state(const Eigen::VectorXd &q_m, const Eigen::VectorXd &dq_m, ros::Time t)
    {
        lfc_msgs::Sensor robot_sensor_msg;
        robot_sensor_msg.header.stamp = t;
        lfc_msgs::Eigen::Sensor robot_sensor_eig;
        robot_sensor_eig.joint_state.position = q_m;
        robot_sensor_eig.joint_state.velocity = dq_m;
        lfc_msgs::sensorEigenToMsg(robot_sensor_eig, robot_sensor_msg);

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
