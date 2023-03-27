#include "panda_torque_mpc/ctrl_mpc_croco.h"

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka/robot_state.h>



namespace panda_torque_mpc
{

    bool CtrlMpcCroco::init(hardware_interface::RobotHW *robot_hw,
                                     ros::NodeHandle &nh)
    {
        ///////////////////
        // Load parameters
        ///////////////////
        std::string arm_id;
        if(!get_param_error_tpl<std::string>(nh, arm_id, "arm_id")) return false;

        // Croco params
        int nb_shooting_nodes, nb_iterations_max;
        double dt_ocp, w_frame_running, w_frame_terminal, w_x_reg_running, w_x_reg_terminal, scale_q_vs_v_reg, w_u_reg_running;
        std::vector<double> armature, diag_u_reg_running;

        if(!get_param_error_tpl<int>(nh, nb_shooting_nodes, "nb_shooting_nodes")) return false;
        if(!get_param_error_tpl<double>(nh, dt_ocp, "dt_ocp")) return false;
        if(!get_param_error_tpl<int>(nh, nb_iterations_max, "nb_iterations_max")) return false;
        if(!get_param_error_tpl<double>(nh, w_frame_running, "w_frame_running")) return false;
        if(!get_param_error_tpl<double>(nh, w_frame_terminal, "w_frame_terminal")) return false;
        if(!get_param_error_tpl<double>(nh, w_x_reg_running, "w_x_reg_running")) return false;
        if(!get_param_error_tpl<double>(nh, w_x_reg_terminal, "w_x_reg_terminal")) return false;
        if(!get_param_error_tpl<double>(nh, scale_q_vs_v_reg, "scale_q_vs_v_reg")) return false;
        if(!get_param_error_tpl<double>(nh, w_u_reg_running, "w_u_reg_running")) return false;

        if(!get_param_error_tpl<std::vector<double>>(nh, armature, "armature", 
                                                          [](std::vector<double> v) {return v.size() == 7;})) return false;
        if(!get_param_error_tpl<std::vector<double>>(nh, diag_u_reg_running, "diag_u_reg_running", 
                                                          [](std::vector<double> v) {return v.size() == 7;})) return false;


        // Panda
        std::vector<std::string> joint_names;
        if(!get_param_error_tpl<std::vector<std::string>>(nh, joint_names, "joint_names", 
                                                          [](std::vector<std::string> v) {return v.size() == 7;})) return false;

        // Trajectory
        std::vector<double> delta_nu;
        if(!get_param_error_tpl<std::vector<double>>(nh, delta_nu, "delta_nu", 
                                                     [](std::vector<double> v) {return v.size() == 6;})) return false;
        delta_nu_ = Eigen::Map<Vector6d>(delta_nu.data());
        std::vector<double> period_nu;
        if(!get_param_error_tpl<std::vector<double>>(nh, period_nu, "period_nu", 
                                                     [](std::vector<double> v) {return v.size() == 6;})) return false;
        period_nu_ = Eigen::Map<Vector6d>(period_nu.data());
        
        // // From a topic? 
        // if(!get_param_error_tpl<bool>(nh, use_external_pose_publisher_, "use_external_pose_publisher")) return false;

        double publish_rate(30.0);
        if (!nh.getParam("publish_rate", publish_rate))
        {
            ROS_INFO_STREAM("CtrlMpcCroco: publish_rate not found. Defaulting to " << publish_rate);
        }
        rate_trigger_ = franka_hw::TriggerRate(publish_rate);

        if(!get_param_error_tpl<double>(nh, alpha_dq_filter_, "alpha_dq_filter")) return false;

        // Load panda model with pinocchio
        std::string urdf_path;
        if(!get_param_error_tpl<std::string>(nh, urdf_path, "urdf_path")) return false;

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
        //                    MPC                      //
        /////////////////////////////////////////////////
        config_croco_.T = nb_shooting_nodes;
        config_croco_.dt_ocp = dt_ocp;
        config_croco_.nb_iterations_max = nb_iterations_max;
        config_croco_.ee_frame_name = ee_frame_pin_;
        config_croco_.w_frame_running = w_frame_running;
        config_croco_.w_frame_terminal = w_frame_terminal;
        config_croco_.w_x_reg_running = w_x_reg_running;
        config_croco_.w_x_reg_terminal = w_x_reg_terminal;
        config_croco_.scale_q_vs_v_reg = scale_q_vs_v_reg;
        config_croco_.w_u_reg_running = w_u_reg_running;
        config_croco_.armature = Eigen::Map<Eigen::Matrix<double,7,1>>(armature.data());
        config_croco_.diag_u_reg_running = Eigen::Map<Eigen::Matrix<double,7,1>>(diag_u_reg_running.data());

        croco_reaching_ = CrocoddylReaching(model_pin_, config_croco_);
        /////////////////////////////////////////////////

        ///////////////////
        // Claim interfaces
        ///////////////////
        // Retrieve resource FrankaStateHandle
        auto *franka_state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
        if (franka_state_interface == nullptr)
        {
            ROS_ERROR("CtrlMpcCroco: Could not get Franka state interface from hardware");
            return false;
        }
        try
        {
            franka_state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(franka_state_interface->getHandle(arm_id + "_robot"));
        }
        catch (const hardware_interface::HardwareInterfaceException &e)
        {
            ROS_ERROR_STREAM("CtrlMpcCroco: Exception getting franka state handle: " << e.what());
            return false;
        }

        // Retrieve resource FrankaModelHandle
        auto *model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
        if (model_interface == nullptr)
        {
            ROS_ERROR_STREAM("CtrlMpcCroco: Error getting model interface from hardware");
            return false;
        }
        try
        {
            franka_model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(model_interface->getHandle(arm_id + "_model"));
        }
        catch (hardware_interface::HardwareInterfaceException &e)
        {
            ROS_ERROR_STREAM("CtrlMpcCroco: Exception getting model handle from interface: " << e.what());
            return false;
        }

        // Retrieve resource FrankaModelHandle
        auto *effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
        if (effort_joint_interface == nullptr)
        {
            ROS_ERROR_STREAM("CtrlMpcCroco: Error getting effort joint interface from hardware");
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
                ROS_ERROR_STREAM("CtrlMpcCroco: Exception getting joint handles: " << e.what());
                return false;
            }
        }

        // Logs publishers
        task_pose_publisher_.init(nh, "task_pose_comparison", 1);
        task_twist_publisher_.init(nh, "task_twist_comparison", 1);
        torques_publisher_.init(nh, "joint_torques_comparison", 1);

        // // Pose subscriber
        // std::string target_pose_topic = "target_pose";
        // if (use_external_pose_publisher_)
        // {
        //     pose_subscriber_ = nh.subscribe(target_pose_topic, 1, &CtrlMpcCroco::pose_callback, this);
        // }

        // init some variables
        dq_filtered_ = Vector7d::Zero();
        // pose_frames_not_aligned_ = true;

        return true;
    }

    void CtrlMpcCroco::starting(const ros::Time &t0)
    {
        ROS_INFO_STREAM("CtrlMpcCroco::starting");
        t_init_ = t0;
        q_init_ = Eigen::Map<const Vector7d>(franka_state_handle_->getRobotState().q.data());
        pin::forwardKinematics(model_pin_, data_pin_, q_init_);
        pin::updateFramePlacements(model_pin_, data_pin_);
        T_b_e0_ = data_pin_.oMf[ee_frame_id_];

        ROS_INFO_STREAM("CtrlMpcCroco::starting T_b_e0_: \n" << T_b_e0_);

        // Set initial goal -> do not move from original pose
        x_r_rtbox_.set(T_b_e0_);
        dx_r_rtbox_.set(pin::Motion::Zero());
        ddx_r_rtbox_.set(pin::Motion::Zero());

        // Set initial posture reference
        Eigen::Matrix<double, 14, 1> x_init;
        x_init << q_init_, Vector7d::Zero();
        croco_reaching_.set_posture_ref(x_init);
    }

    void CtrlMpcCroco::update(const ros::Time &t, const ros::Duration &period)
    {
        TicTac tictac;

        // Time since start of the controller
        double Dt = (t - t_init_).toSec();

        // Retrieve reference
        pin::SE3 x_r; 
        pin::Motion dx_r, ddx_r;
        compute_sinusoid_pose_reference(delta_nu_, period_nu_, T_b_e0_, Dt, x_r, dx_r, ddx_r);
        x_r_rtbox_.set(x_r); dx_r_rtbox_.set(dx_r); ddx_r_rtbox_.set(ddx_r);

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

        // Compute desired torque
        TicTac tictac_comp;
        Vector7d tau_d = compute_desired_torque(q_m, dq_m, dq_filtered_, x_r, config_croco_);
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
        last_x_r_ = x_r;
        last_dx_r_ = dx_r;
        last_tau_d_ = tau_d_saturated + Eigen::Map<Vector7d>(franka_model_handle_->getGravity().data());

        tictac.print_tac("update() took (ms): ");
    }

    Vector7d CtrlMpcCroco::compute_desired_torque(
        const Vector7d &q_m, const Vector7d &dq_m, const Vector7d &dq_filtered, const pin::SE3 &x_r, const CrocoddylConfig& conf)
    {

        // pin::rnea(model_pin_, data_pin_, q_m, dq_m, ddq_d);
        pin::computeGeneralizedGravity(model_pin_, data_pin_, q_m); // data.g == generalized gravity
        // Vector7d tau_grav = pin::computeGeneralizedGravity(model_pin_, data_pin_, q_m);
        Vector7d tau_grav = data_pin_.g;

        // Initialise the MPC solver from current state ("closed loop" MPC)
        Eigen::Matrix<double, 14, 1> x_m;
        x_m << q_m, dq_m;

        std::vector<Eigen::Matrix<double, -1, 1>> xs_init;
        std::vector<Eigen::Matrix<double, -1, 1>> us_init;
        

        // !!!!!!!!!!!!!!!
        // goal_translation_set_ is used to detect if a problem has been already solved
        // A bit werid
        // !!!!!!!!!!!!!!!
        if (!croco_reaching_.goal_translation_set_)
        {
            // if first occurence, use a sensible prior (no movement and gravity compensation)
            xs_init.reserve(config_croco_.T);
            us_init.reserve(config_croco_.T);
            // x_init is size T+1
            // u_init is size T (terminal node has no control variable attached)
            for (int i = 0; i < config_croco_.T; i++)
            {
                xs_init.push_back(x_m);
                us_init.push_back(tau_grav);
            }
            xs_init.push_back(x_m);
        }
        else
        {
            // Retrieve previous trajectory and shift by one timestep
            xs_init = croco_reaching_.ddp_->get_xs();
            us_init = croco_reaching_.ddp_->get_us();

            // If the update period and OCP shooting dt are too different
            // maybe it does not make sense to shift...
            // xs_init.insert(std::begin(xs_init), xs_init.at(0));
            // xs_init.erase( std::end(xs_init)-1);
            // us_init.insert(std::begin(us_init), us_init.at(0));
            // us_init.erase( std::end(us_init)-1);
        }

        // Set initial state and end-effector ref
        croco_reaching_.ddp_->get_problem()->set_x0(x_m);
        croco_reaching_.set_ee_ref(x_r.translation());

        croco_reaching_.ddp_->solve(xs_init, us_init, conf.nb_iterations_max, false);

        Vector7d tau_d = croco_reaching_.ddp_->get_us()[0] - tau_grav;

        return tau_d;
    }

    void CtrlMpcCroco::compute_sinusoid_pose_reference(const Vector6d &delta_nu, const Vector6d &period_nu, const pin::SE3 &pose_0, double t,
                                                       pin::SE3 &x_r, pin::Motion &dx_r, pin::Motion &ddx_r)
    {
        // Ai and Ci obtained for each joint using constraints:
        // T(t=0.0) = pose_0
        // T(t=period/2) = pose_0 * Exp(delta_nu)

        Vector6d w = (2 * M_PI / period_nu.array()).matrix();
        Vector6d a = -delta_nu;
        Vector6d c = delta_nu;

        Vector6d nu = (a.array() * cos(w.array() * t)).matrix() + c;
        dx_r = pin::Motion((-w.array() * a.array() * sin(w.array() * t)).matrix());
        ddx_r = pin::Motion((-w.array().square() * a.array() * cos(w.array() * t)).matrix()); // non null initial acceleration!! needs to be dampened (e.g. torque staturation)

        x_r = pose_0 * pin::exp6(nu);
    }

    // void CtrlMpcCroco::pose_callback(const PoseTaskGoal& msg)
    // {   

    //     /**
    //      * T_a_b: SE3 transformation from frame b to a, a_vec = T_a_b * b_vec
    //      * 
    //      * Frames:
    //      * - w: "world" reference frame of the pose message
    //      * - t: "target" frame of the pose message
    //      * - b: "base" frame of the robot (root of the kinematic tree)
    //      * - e: "end" effector of the robot
    //     */

    //     std::cout << "CtrlMpcCroco::pose_callback PoseTaskGoal:" << std::endl;
    //     std::cout << msg.pose.position.x << std::endl;
    //     std::cout << msg.pose.position.y << std::endl;
    //     std::cout << msg.pose.position.z << std::endl;
    //     std::cout << msg.pose.orientation.x << std::endl;
    //     std::cout << msg.pose.orientation.y << std::endl;
    //     std::cout << msg.pose.orientation.z << std::endl;
    //     std::cout << msg.pose.orientation.w << std::endl;

    //     Eigen::Vector3d t_bt; t_bt << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;
    //     Eigen::Quaterniond quat_bt(msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z);
    //     pin::SE3 T_w_t(quat_bt, t_bt);

    //     if (pose_frames_not_aligned_)
    //     {
    //         T_w_t0_ = T_w_t;
    //         pose_frames_not_aligned_ = false;
    //     }

    //     // We want to apply to the robot end-effector the same transformation
    //     // as the target pose in the initial pose of the target/robot
    //     // pin::SE3 T_e0_e = T_w_t0_*T_w_t;
    //     pin::SE3 T_e0_e = pin::SE3::Identity();
    //     auto R_e0_b = T_b_e0_.rotation().transpose();
    //     // Align w and b frames -> assume w = b
    //     T_e0_e.translation() = R_e0_b * (T_w_t.translation() - T_w_t0_.translation());


    //     // std::cout << T_w_t.translation().transpose() << std::endl;
    //     // std::cout << T_w_t0_.translation().transpose() << std::endl;
    //     // std::cout << T_e0_e.translation().transpose() << std::endl;


    //     // Set reference po se
    //     // compose initial pose with relative/local transform
    //     pin::SE3 T_be = T_b_e0_*T_e0_e;
    //     std::cout << "callback T_be trans" << T_be.translation().transpose() << std::endl;

    //     pin::Motion nu_wt;
    //     // Set reference twist
    //     nu_wt.linear().x() = msg.twist.linear.x;
    //     nu_wt.linear().y() = msg.twist.linear.y;
    //     nu_wt.linear().z() = msg.twist.linear.z;
    //     nu_wt.angular().x() = msg.twist.angular.x;
    //     nu_wt.angular().y() = msg.twist.angular.y;
    //     nu_wt.angular().z() = msg.twist.angular.z;
    //     // Set reference spatial acceleration
    //     pin::Motion a_wt;
    //     a_wt.linear().x() = msg.acceleration.linear.x;
    //     a_wt.linear().y() = msg.acceleration.linear.y;
    //     a_wt.linear().z() = msg.acceleration.linear.z;
    //     a_wt.angular().x() = msg.acceleration.angular.x;
    //     a_wt.angular().y() = msg.acceleration.angular.y;
    //     a_wt.angular().z() = msg.acceleration.angular.z;
    //     // RT safe setting
    //     x_r_rtbox_.set(T_be);
    //     dx_r_rtbox_.set(nu_wt);
    //     ddx_r_rtbox_.set(a_wt);
    // }

    void CtrlMpcCroco::stopping(const ros::Time &t0)
    {
        ROS_INFO_STREAM("CtrlMpcCroco::stopping");
    }

} // namespace panda_torque_mpc

PLUGINLIB_EXPORT_CLASS(panda_torque_mpc::CtrlMpcCroco,
                       controller_interface::ControllerBase)
