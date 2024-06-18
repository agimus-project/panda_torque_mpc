#include "panda_torque_mpc/ctrl_task_space_ID.h"

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka/robot_state.h>


namespace panda_torque_mpc
{

    bool CtrlTaskSpaceID::init(hardware_interface::RobotHW *robot_hw,
                                     ros::NodeHandle &nh)
    {

        ///////////////////
        // Load parameters
        ///////////////////
        std::string arm_id;
        if(!get_param_error_tpl<std::string>(nh, arm_id, "arm_id")) return false;

        std::vector<std::string> joint_names;
        if(!get_param_error_tpl<std::vector<std::string>>(nh, joint_names, "joint_names", 
                                                          [](std::vector<std::string> v) {return v.size() == 7;})) return false;

        if(!get_param_error_tpl<double>(nh, kp_ee_, "kp_ee")) return false;
        if(!get_param_error_tpl<double>(nh, kd_ee_, "kd_ee")) return false;
        if(!get_param_error_tpl<double>(nh, kp_q_,  "kp_q"))  return false;
        if(!get_param_error_tpl<double>(nh, kd_ee_, "kd_q"))  return false;
        if(!get_param_error_tpl<double>(nh, w_ee_, "w_ee"))  return false;
        if(!get_param_error_tpl<double>(nh, w_q_,  "w_q"))  return false;
        if(!get_param_error_tpl<double>(nh, w_q_,  "w_q"))  return false;
        if(!get_param_error_tpl<double>(nh, tau_limit_scale_, "tau_limit_scale"))  return false;
        if(!get_param_error_tpl<double>(nh, v_limit_scale_,   "v_limit_scale"))  return false;
        

        std::vector<double> ee_task_mask_vec;
        if(!get_param_error_tpl<std::vector<double>>(nh, ee_task_mask_vec, "ee_task_mask", 
                                                     [](std::vector<double> v) {return v.size() == 6;})) return false;
        ee_task_mask_ = Eigen::Map<Vector6d>(ee_task_mask_vec.data());

        // Trajectory
        std::vector<double> delta_nu;
        if(!get_param_error_tpl<std::vector<double>>(nh, delta_nu, "delta_nu", 
                                                     [](std::vector<double> v) {return v.size() == 6;})) return false;
        delta_nu_ = Eigen::Map<Vector6d>(delta_nu.data());
        std::vector<double> period_nu;
        if(!get_param_error_tpl<std::vector<double>>(nh, period_nu, "period_nu", 
                                                     [](std::vector<double> v) {return v.size() == 6;})) return false;
        period_nu_ = Eigen::Map<Vector6d>(period_nu.data());
        
        // From a topic? 
        if(!get_param_error_tpl<bool>(nh, use_external_pose_publisher_, "use_external_pose_publisher")) return false;

        double publish_rate(30.0);
        if (!nh.getParam("publish_rate", publish_rate))
        {
            ROS_INFO_STREAM("CtrlTaskSpaceID: publish_rate not found. Defaulting to " << publish_rate);
        }
        rate_trigger_ = franka_hw::TriggerRate(publish_rate);

         int idc;
         if (!nh.getParam("control_variant", idc) || !(idc >= 0 && idc < 3))
         {
             ROS_ERROR_STREAM("CtrlTaskSpaceID: Invalid or no control_variant parameters provided, aborting controller init! control_variant: " << idc);
         }
         control_variant_ = static_cast<CtrlTaskSpaceID::TSIDVariant>(idc);
 
        if(!get_param_error_tpl<bool>(nh, use_pinocchio_, "use_pinocchio")) return false;
        if(!get_param_error_tpl<double>(nh, alpha_dq_filter_, "alpha_dq_filter")) return false;

        // Load panda model with pinocchio
        if(!get_param_error_tpl<std::string>(nh, ee_frame_name_, "ee_frame_name")) return false;

        std::string robot_description;
        if(!get_param_error_tpl<std::string>(nh, robot_description, "/robot_description")) return false;

        model_pin_ = loadPandaPinocchio(robot_description);
        data_pin_ = pin::Data(model_pin_);

        if ((model_pin_.nq != 7) || (model_pin_.name != "panda"))
        {
            ROS_ERROR_STREAM("Problem when loading the robot urdf");
            return false;
        }

        // Define corresponding frame id for pinocchio and Franka (see ctrl_model_pinocchio_vs_franka)
        ee_frame_id_ = model_pin_.getFrameId(ee_frame_name_);

        /////////////////////////////////////////////////
        //                    TSID                     //
        /////////////////////////////////////////////////
        TsidConfig conf;
        conf.kp_ee = kp_ee_;  
        conf.kd_ee = kd_ee_;  
        conf.kp_q = kp_q_; 
        conf.kd_q = kd_q_; 
        conf.w_ee = w_ee_; 
        conf.w_q = w_q_; 
        conf.tau_limit_scale = tau_limit_scale_;
        conf.v_limit_scale = v_limit_scale_;
        conf.ee_frame_name = ee_frame_name_;
        conf.ee_task_mask = ee_task_mask_;
        tsid_reaching_ = TsidManipulatorReaching(model_pin_, conf);
        /////////////////////////////////////////////////

        ///////////////////
        // Claim interfaces
        ///////////////////
        // Retrieve resource FrankaStateHandle
        auto *franka_state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
        if (franka_state_interface == nullptr)
        {
            ROS_ERROR("CtrlTaskSpaceID: Could not get Franka state interface from hardware");
            return false;
        }
        try
        {
            franka_state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(franka_state_interface->getHandle(arm_id + "_robot"));
        }
        catch (const hardware_interface::HardwareInterfaceException &e)
        {
            ROS_ERROR_STREAM("CtrlTaskSpaceID: Exception getting franka state handle: " << e.what());
            return false;
        }

        // Retrieve resource FrankaModelHandle
        auto *model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
        if (model_interface == nullptr)
        {
            ROS_ERROR_STREAM("CtrlTaskSpaceID: Error getting model interface from hardware");
            return false;
        }
        try
        {
            franka_model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(model_interface->getHandle(arm_id + "_model"));
        }
        catch (hardware_interface::HardwareInterfaceException &e)
        {
            ROS_ERROR_STREAM("CtrlTaskSpaceID: Exception getting model handle from interface: " << e.what());
            return false;
        }

        // Retrieve resource FrankaModelHandle
        auto *effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
        if (effort_joint_interface == nullptr)
        {
            ROS_ERROR_STREAM("CtrlTaskSpaceID: Error getting effort joint interface from hardware");
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
                ROS_ERROR_STREAM("CtrlTaskSpaceID: Exception getting joint handles: " << e.what());
                return false;
            }
        }

        // Logs publishers
        task_pose_publisher_.init(nh, "task_pose_comparison", 1);
        task_twist_publisher_.init(nh, "task_twist_comparison", 1);
        torques_publisher_.init(nh, "joint_torques_comparison", 1);

        // Pose subscriber
        std::string motion_capture_pose_ref_topic = "motion_capture_pose_ref";
        if (use_external_pose_publisher_)
        {
            motion_capture_pose_ref_subscriber_ = nh.subscribe(motion_capture_pose_ref_topic, 1, &CtrlTaskSpaceID::pose_callback, this);
        }

        // init some variables
        dq_filtered_ = Vector7d::Zero();
        pose_frames_not_aligned_ = true;

        return true;
    }

    void CtrlTaskSpaceID::starting(const ros::Time &t0)
    {
        ROS_INFO_STREAM("CtrlTaskSpaceID::starting");
        t_init_ = t0;
        q_init_ = Eigen::Map<const Vector7d>(franka_state_handle_->getRobotState().q.data());
        pin::forwardKinematics(model_pin_, data_pin_, q_init_);
        pin::updateFramePlacements(model_pin_, data_pin_);
        T_b_e0_ = data_pin_.oMf[ee_frame_id_];

        // Set posture reference once and for all
        tsid_reaching_.setPostureRef(q_init_);

        ROS_INFO_STREAM("CtrlTaskSpaceID::starting T_b_e0_: \n" << T_b_e0_);

        // Set initial goal -> do not move from original pose
        x_r_rtbox_.set(T_b_e0_);
        dx_r_rtbox_.set(pin::Motion::Zero());
        ddx_r_rtbox_.set(pin::Motion::Zero());
    }

    void CtrlTaskSpaceID::update(const ros::Time &t, const ros::Duration &period)
    {
        TicTac tictac;

        // Time since start of the controller
        double Dt = (t - t_init_).toSec();

        // Instanciate reference pose variables
        pin::SE3 x_r; 
        pin::Motion dx_r, ddx_r;

        // compute end effector reference if no topic reference exists
        if (!use_external_pose_publisher_)
        {
            compute_sinusoid_pose_reference(delta_nu_, period_nu_, T_b_e0_, Dt, x_r, dx_r, ddx_r);
            x_r_rtbox_.set(x_r); dx_r_rtbox_.set(dx_r); ddx_r_rtbox_.set(ddx_r);
        }    

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

        TicTac tictac_comp;
        // Compute desired torque according to current stored reference
        x_r_rtbox_.get(x_r); dx_r_rtbox_.get(dx_r); ddx_r_rtbox_.get(ddx_r);
        Vector7d tau_d = compute_desired_torque(q_m, dq_m, dq_filtered_, x_r, dx_r, ddx_r, control_variant_, use_pinocchio_);
        tictac_comp.print_tac("compute_desired_torque() took (ms): ");

        // Maximum torque difference with a sampling rate of 1 kHz. The maximum torque rate is
        // 1000 * (1 / sampling_time).
        // Vector7d tau_d_saturated = saturateTorqueRate(tau_d, Eigen::Map<Vector7d>(robot_state.tau_J_d.data()));
        Vector7d tau_d_saturated = tau_d;

        // Send Torque Command
        for (size_t i = 0; i < 7; ++i)
        {
            joint_handles_[i].setCommand(tau_d_saturated[i]);
        }

        ///////////////////
        // Publish logs
        // Takes about 5 us -> negligeable
        if (rate_trigger_() && torques_publisher_.trylock())
        {
            // TicTac tt_publish;
            // Refactor to: publish_logs(publishers, last_vals, measured_vals) ?

            /**
             * Measured torque in simulation returns -tau while running with real robot returns tau --___--
             */
            tau_m = -tau_m; // SIMULATION
            // tau_m = tau_m;  // REAL

            // torque
            Vector7d tau_error = last_tau_d_ - tau_m;
            // EE pose
            Eigen::Vector3d p_o_e_err = T_o_e_m.translation() - last_x_r_.translation();
            Eigen::Quaterniond quat_r(last_x_r_.rotation());
            Eigen::Quaterniond quat_m(T_o_e_m.rotation());
            Eigen::Quaterniond quat_err = quat_r.inverse() * quat_m;
            // EE twist
            Eigen::Vector3d v_o_e_err = nu_o_e_m.linear() - dx_r.linear();
            Eigen::Vector3d omg_o_e_err = nu_o_e_m.angular() - dx_r.angular();


            // EE Twists linear part
            task_twist_publisher_.msg_.commanded.linear.x = dx_r.linear()[0];
            task_twist_publisher_.msg_.commanded.linear.y = dx_r.linear()[1];
            task_twist_publisher_.msg_.commanded.linear.z = dx_r.linear()[2];
            task_twist_publisher_.msg_.measured.linear.x = nu_o_e_m.linear()[0];
            task_twist_publisher_.msg_.measured.linear.y = nu_o_e_m.linear()[1];
            task_twist_publisher_.msg_.measured.linear.z = nu_o_e_m.linear()[2];
            task_twist_publisher_.msg_.error.linear.x = v_o_e_err[0];
            task_twist_publisher_.msg_.error.linear.y = v_o_e_err[1];
            task_twist_publisher_.msg_.error.linear.z = v_o_e_err[2];

            // EE Twists angular part
            task_twist_publisher_.msg_.commanded.angular.x = dx_r.angular()[0];
            task_twist_publisher_.msg_.commanded.angular.y = dx_r.angular()[1];
            task_twist_publisher_.msg_.commanded.angular.z = dx_r.angular()[2];
            task_twist_publisher_.msg_.measured.angular.x = nu_o_e_m.angular()[0];
            task_twist_publisher_.msg_.measured.angular.y = nu_o_e_m.angular()[1];
            task_twist_publisher_.msg_.measured.angular.z = nu_o_e_m.angular()[2];
            task_twist_publisher_.msg_.error.angular.x = omg_o_e_err[0];
            task_twist_publisher_.msg_.error.angular.y = omg_o_e_err[1];
            task_twist_publisher_.msg_.error.angular.z = omg_o_e_err[2];

            // End effector position
            task_pose_publisher_.msg_.commanded.position.x = x_r.translation()[0];
            task_pose_publisher_.msg_.commanded.position.y = x_r.translation()[1];
            task_pose_publisher_.msg_.commanded.position.z = x_r.translation()[2];
            task_pose_publisher_.msg_.measured.position.x = T_o_e_m.translation()[0];
            task_pose_publisher_.msg_.measured.position.y = T_o_e_m.translation()[1];
            task_pose_publisher_.msg_.measured.position.z = T_o_e_m.translation()[2];
            task_pose_publisher_.msg_.error.position.x = p_o_e_err[0];
            task_pose_publisher_.msg_.error.position.y = p_o_e_err[1];
            task_pose_publisher_.msg_.error.position.z = p_o_e_err[2];

            // Quaternion
            task_pose_publisher_.msg_.commanded.orientation.x = quat_r.x();
            task_pose_publisher_.msg_.commanded.orientation.y = quat_r.y();
            task_pose_publisher_.msg_.commanded.orientation.z = quat_r.z();
            task_pose_publisher_.msg_.commanded.orientation.w = quat_r.w();
            task_pose_publisher_.msg_.measured.orientation.x = quat_m.x();
            task_pose_publisher_.msg_.measured.orientation.y = quat_m.y();
            task_pose_publisher_.msg_.measured.orientation.z = quat_m.z();
            task_pose_publisher_.msg_.measured.orientation.w = quat_m.w();
            task_pose_publisher_.msg_.error.orientation.x = quat_err.x();
            task_pose_publisher_.msg_.error.orientation.y = quat_err.y();
            task_pose_publisher_.msg_.error.orientation.z = quat_err.z();
            task_pose_publisher_.msg_.error.orientation.w = quat_err.w();

            // Size 7 vectors
            for (size_t i = 0; i < 7; ++i)
            {
                torques_publisher_.msg_.commanded[i] = last_tau_d_[i];
                torques_publisher_.msg_.measured[i] = tau_m[i];
                torques_publisher_.msg_.error[i] = tau_error[i];
            }

            task_pose_publisher_.unlockAndPublish();
            task_twist_publisher_.unlockAndPublish();
            torques_publisher_.unlockAndPublish();

            // std::cout << std::setprecision(9) << "publish took (ms): " << tt_publish.tac() << std::endl;
        }

        // Store previous desired/reference values
        x_r_rtbox_.get(last_x_r_);
        dx_r_rtbox_.get(last_dx_r_);
        last_tau_d_ = tau_d_saturated + Eigen::Map<Vector7d>(franka_model_handle_->getGravity().data());

        tictac.print_tac("update() took (ms): ");
    }

    Vector7d CtrlTaskSpaceID::compute_desired_torque(
        const Vector7d &q_m, const Vector7d &dq_m, const Vector7d &dq_filtered,
        const pin::SE3 &x_r, const pin::Motion &dx_r, const pin::Motion &ddx_r,
        TSIDVariant control_variant, bool use_pinocchio)
    {

        /**
         *
         * Lagrangian dynamics:
         * M(q)*ddq + C(q,dq)*dq + g(q) = tau + J^T*f
         *
         * Here, we assume no contact so f=0.
         *
         *
         * TSID -> feedback at the task acceleration level (end effector reference tracking)
         * ddx_d = ddx_r − Kd ( dx_m − dx_r) − Kp(x_m − x_r)
         * where dx_m and x_m come from Forward kinematics
         *
         * if x is R^3 (EE position control) -> x_m - x_r: simple euclidean diff
         * BUT, if x is SE3 (EE pose control) -> x_m - x_r = log(x_r^-1 * x)
         *
         * ddq_d = J^# (ddx_d - dJ*dq_m)
         * Or solve
         * J*ddq_d = ddx_d - dJ*dq_m
         * tau_d = M ddq_d + h(q_m, dq_m) = rnea(q_m, dq_m, ddq_d)
         */

        /**
         * For differential Forward Kinematics and jacobians, use LOCAL_WORLD_ALIGNED
         * for all computations since corresponds to the "classical" euclidean velocity (easier to think about)
         *
         */
        // FK and differential FK
        pin::forwardKinematics(model_pin_, data_pin_, q_m, dq_m);
        pin::updateFramePlacements(model_pin_, data_pin_);
        pin::SE3 T_o_e_m = data_pin_.oMf[ee_frame_id_];
        pin::Motion nu_o_e_m = pin::getFrameVelocity(model_pin_, data_pin_, ee_frame_id_, pin::LOCAL_WORLD_ALIGNED);

        // end effector jacobian and time derivative
        Eigen::Matrix<double, 6, 7> J_pin, dJ_pin;
        J_pin.setZero();
        pin::computeFrameJacobian(model_pin_, data_pin_, q_m, ee_frame_id_, pin::LOCAL_WORLD_ALIGNED, J_pin);
        dJ_pin.setZero();
        pin::computeJointJacobiansTimeVariation(model_pin_, data_pin_, q_m, dq_m);
        pin::getFrameJacobianTimeVariation(model_pin_, data_pin_, ee_frame_id_, pin::LOCAL_WORLD_ALIGNED, dJ_pin);

        Vector7d ddq_d; // desired joint acceleration
        switch (control_variant)
        {
        case TSIDVariant::PosiPosture:
        {
            // ROS_INFO_STREAM("TSIDVariant::PosiPosture, pinocchio: " << use_pinocchio_);
            ////////////
            // EE POSITION + POSTURE tasks
            // Position task
            Eigen::Vector3d e_x = T_o_e_m.translation() - x_r.translation();
            Eigen::Vector3d de_x = nu_o_e_m.linear() - dx_r.linear();
            Eigen::Vector3d ddx_d = ddx_r.linear() - kp_ee_ * e_x - kd_ee_ * de_x;

            // Posture task : q --> q_init
            // Let's keep the same dynamics but alpha will handle the weighting between the 2 tasks
            // ddq_r = 0 = dq_r here
            Vector7d eq = q_m - q_init_;
            Vector7d deq = dq_m;
            Vector7d ddq_reg_d = -kp_q_*eq - kd_q_*deq;

            // Create and solve least square problem to get desired joint acceleration
            Eigen::Matrix<double, 10, 7> A;
            Eigen::Matrix<double, 3, 7> J_pin_p = J_pin.block<3, 7>(0, 0);
            Eigen::Matrix<double, 3, 7> dJ_pin_p = dJ_pin.block<3, 7>(0, 0);
            A.block<3, 7>(0, 0) = pow(w_ee_, 2) * J_pin_p;
            A.block<7, 7>(3, 0) = pow(w_q_, 2) * Eigen::Matrix<double, 7, 7>::Identity();
            Eigen::Matrix<double, 10, 1> b;
            b.segment<3>(0) = pow(w_ee_, 2) * (ddx_d - dJ_pin_p * dq_m);
            b.segment<7>(3) = pow(w_q_, 2) * ddq_reg_d;

            ddq_d = A.colPivHouseholderQr().solve(b);

            break;
        }
        case TSIDVariant::PosePosture:
        {
            // ROS_INFO_STREAM("TSIDVariant::PosePosture, pinocchio: " << use_pinocchio_);

            // EE SE3 POSE + POSTURE tasks  --> NOPE, likely problem with the jacobian computation

            // 1) End effector pose taskquat_r_local
            // !! we are in LOCAL_WORLD_ALIGNED mode
            // --> task error needs to be coherent!
            // e_T = minus(Tm, Tref) = log6(Tm^-1 * Tref) or log6(Tref^-1 * Tm) depending on:
            // pin::Motion e_x = pin::log6(x_r.inverse() * T_o_e_m);  // WORLD se3 cartesian tangent space difference  -> NOPE
            // pin::Motion e_x = pin::log6(T_o_e_m.inverse() * x_r);  // LOCAL se3 cartesian tangent space difference  -> NOPE

            // LOCAL_WORLD_ALIGNED se3 cartesian tangent space difference
            // e_t = minus(tm, tref) = tm - tref   by definition of SO(3) manifold difference operator
            // e_R = minus(Rm, Rref) = log3(Rref.T * Rm)   by definition of SO(3) manifold difference operator
            pin::Motion e_x = pin::Motion::Zero();
            e_x.linear() = T_o_e_m.translation() - x_r.translation();
            // e_x.angular() = pin::log3(x_r.rotation().transpose() * T_o_e_m.rotation());  // in world frame!
            // e_x.angular() = pin::log3(T_o_e_m.rotation().transpose() * x_r.rotation());  // in body frame!
            e_x.angular() = Eigen::Vector3d::Zero();  // HACK!! two above do NOT work!!
            pin::Motion de_x = nu_o_e_m - dx_r;

            // A pin::Motion behaves like a Eigen::Vector6d in this case
            Vector6d ddx_d = ddx_r - kp_ee_ * e_x - kd_ee_ * de_x;

            // 2) Posture task : q --> q_init
            // Let's keep the same dynamics but alpha will handle the weighting between the 2 tasks
            // ddq_r = 0 = dq_r here
            Vector7d eq = q_m - q_init_;
            Vector7d deq = dq_m;
            Vector7d ddq_reg_d = -kp_ee_ * eq - kd_ee_ * deq;

            // SE3 ONLY -> UNDERDETERMINED
            // Eigen::Matrix<double, 6, 7> A = Jlog * J_pin;
            // Eigen::Matrix<double, 6, 7> b = ddx_d - dJ_pin * dq_m;
            // ddq_d = A.colPivHouseholderQr().solve(b);

            // Create and solve least square problem to get desired joint acceleration
            Eigen::Matrix<double, 13, 7> A;
            A.block<6, 7>(0, 0) = pow(w_ee_, 2) * J_pin;
            A.block<7, 7>(6, 0) = pow(w_q_, 2) * Eigen::Matrix<double, 7, 7>::Identity();
            Eigen::Matrix<double, 13, 1> b;
            b.segment<6>(0) = pow(w_ee_, 2) * (ddx_d - dJ_pin * dq_m);
            b.segment<7>(6) = pow(w_q_, 2) * ddq_reg_d;
            ddq_d = A.colPivHouseholderQr().solve(b);

            break;
        }
        case TSIDVariant::TSIDPose:
        {
            // ROS_INFO_STREAM("TSIDVariant::TSID, pinocchio: " << use_pinocchio_);

            tsid_reaching_.setEERef(x_r, dx_r, ddx_r);
            tsid_reaching_.solve(q_m, dq_m);
            // ddq_d = tsid_reaching_.getAccelerations();
            Eigen::VectorXd tau_d = tsid_reaching_.getTorques();

            // TSID soft directly returns torques as a result of the optim, just use that (minus gravity compensation)
            pin::computeGeneralizedGravity(model_pin_, data_pin_, q_m); // data.g == generalized gravity
            return tau_d - data_pin_.g;

            break;
        }
        }  // end of the switch

        Vector7d tau_d;
        if (use_pinocchio_)
        {
            pin::rnea(model_pin_, data_pin_, q_m, dq_m, ddq_d);
            pin::computeGeneralizedGravity(model_pin_, data_pin_, q_m); // data.g == generalized gravity

            tau_d = data_pin_.tau - data_pin_.g;
        }

        else
        {
            // libfranka dynamics
            std::array<double, 7> coriolis_fra_arr = franka_model_handle_->getCoriolis();
            std::array<double, 7> gravity_fra_arr = franka_model_handle_->getGravity();
            std::array<double, 49> M_fra_arr = franka_model_handle_->getMass();
            // Eigen and Franka use Column-Major storage order (see model_pinocchio_vs_frank_controller)
            Eigen::Map<Vector7d> coriolis_fra(coriolis_fra_arr.data()); // C(q,dq)*dq
            Eigen::Map<Vector7d> gravity_fra(gravity_fra_arr.data());   // g(q)
            Eigen::Map<Matrix7d> M_fra(M_fra_arr.data());               // M(q)

            tau_d = M_fra * ddq_d + coriolis_fra;
        }

        return tau_d;
    }

    void CtrlTaskSpaceID::pose_callback(const geometry_msgs::PoseStamped& msg)
    {   

        /**
         * T_a_b: SE3 transformation from frame b to a, a_vec = T_a_b * b_vec
         * 
         * Frames:
         * - w: "world" reference frame of the pose message
         * - t: "target" frame of the pose message
         * - b: "base" frame of the robot (root of the kinematic tree)
         * - e: "end" effector of the robot
        */


        pin::SE3 T_w_t = posemsg2SE3(msg.pose);

        // std::cout << "CtrlTaskSpaceID::pose_callback T_w_t:" << std::endl;
        // std::cout << T_w_t << std::endl;

        if (pose_frames_not_aligned_)
        {
            T_w_t0_ = T_w_t;
            pose_frames_not_aligned_ = false;
        }

        /**
         * Problem formulation
         * 
         * We want to apply to the robot end-effector the same motion as the camera/target
         * with respect to the initial time 0.
         * 
         * Given T_b_e0, T_w_t0_ (recorded at initial time), T_w_t, T_b_e (latest values), 
         * define T_e0_e so that the composition  
         * 
         * T_be = T_b_e0 * T_e0_e
         * 
         * produces interesting motion.
         * 
         * 3 ideas:
        */

        // // Idea 1: T_e0_e := T_t0_t
        // //   -> BAD: coupled tranlsation and rotation, produces unintuitive motion  
        // pin::SE3 T_e0_e = T_w_t0_*T_w_t;

        // Idea 2: b_p_e0_e := w_p_t0_t  and    R_e0_e := I_3
        //   -> equal relative positions, gives good results but only position target is given
        pin::SE3 T_e0_e = pin::SE3::Identity();
        auto R_e0_b = T_b_e0_.rotation().transpose();
        T_e0_e.translation() = R_e0_b * (T_w_t.translation() - T_w_t0_.translation());

        // // Idea 3: b_p_e0_e := w_p_t0_t  and    R_e0_e := R_t0_b*R_b_t
        // pin::SE3 T_e0_e = pin::SE3::Identity();
        // auto R_e0_b = T_b_e0_.rotation().transpose();
        // T_e0_e.translation() = R_e0_b * (T_w_t.translation() - T_w_t0_.translation());
        // T_e0_e.rotation() = T_w_t0_.rotation().transpose() * T_w_t.rotation();

        // Set reference pose
        // compose initial pose with relative/local transform
        pin::SE3 T_be = T_b_e0_*T_e0_e;

        pin::Motion nu_wt = pin::Motion::Zero();
        pin::Motion a_wt = pin::Motion::Zero();

        // RT safe setting
        x_r_rtbox_.set(T_be);
        dx_r_rtbox_.set(nu_wt);
        ddx_r_rtbox_.set(a_wt);
    }

    void CtrlTaskSpaceID::stopping(const ros::Time &t0)
    {
        ROS_INFO_STREAM("CtrlTaskSpaceID::stopping");
    }

} // namespace panda_torque_mpc

PLUGINLIB_EXPORT_CLASS(panda_torque_mpc::CtrlTaskSpaceID,
                       controller_interface::ControllerBase)
