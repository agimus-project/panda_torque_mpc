#include "panda_torque_mpc/ctrl_joint_space_ID.h"

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka/robot_state.h>

namespace panda_torque_mpc
{

    bool CtrlJointSpaceID::init(hardware_interface::RobotHW *robot_hw,
                                      ros::NodeHandle &nh)
    {



        std::string arm_id;
        if(!get_param_error_tpl<std::string>(nh, arm_id, "arm_id")) return false;

        std::vector<std::string> joint_names;
        if(!get_param_error_tpl<std::vector<std::string>>(nh, joint_names, "joint_names", 
                                                          [](std::vector<std::string> v) {return v.size() == 7;})) return false;

        if(!get_param_error_tpl<double>(nh, Kp_, "Kp")) return false;
        if(!get_param_error_tpl<double>(nh, Kd_, "Kd")) return false;

        std::vector<double> kp_gains;
        if(!get_param_error_tpl<std::vector<double>>(nh, kp_gains, "kp_gains", 
                                                     [](std::vector<double> v) {return v.size() == 7;})) return false;

        std::vector<double> kd_gains;
        if(!get_param_error_tpl<std::vector<double>>(nh, kd_gains, "kd_gains", 
                                                     [](std::vector<double> v) {return v.size() == 7;})) return false;


        std::vector<double> delta_q;
        if(!get_param_error_tpl<std::vector<double>>(nh, delta_q, "delta_q", 
                                                     [](std::vector<double> v) {return v.size() == 7;})) return false;


        std::vector<double> period_q;
        if(!get_param_error_tpl<std::vector<double>>(nh, period_q, "period_q", 
                                                     [](std::vector<double> v) {return v.size() == 7;})) return false;

        int idc;
        if (!nh.getParam("control_variant", idc) || !(idc >= 0 && idc < 4))
        {
            ROS_ERROR_STREAM("CtrlJointSpaceID: Invalid or no control_variant parameters provided, aborting controller init! control_variant: " << idc);
        }
        control_variant_ = static_cast<CtrlJointSpaceID::JSIDVariant>(idc);

        kp_gains_ = Eigen::Map<Vector7d>(kp_gains.data());
        kd_gains_ = Eigen::Map<Vector7d>(kd_gains.data());
        delta_q_ = Eigen::Map<Vector7d>(delta_q.data());
        period_q_ = Eigen::Map<Vector7d>(period_q.data());

        double publish_rate(30.0);
        if (!nh.getParam("publish_rate", publish_rate))
        {
            ROS_INFO_STREAM("CtrlJointSpaceID: publish_rate not found. Defaulting to " << publish_rate);
        }
        rate_trigger_ = franka_hw::TriggerRate(publish_rate);


        if(!get_param_error_tpl<bool>(nh, use_pinocchio_, "use_pinocchio")) return false;
        if(!get_param_error_tpl<double>(nh, alpha_dq_filter_, "alpha_dq_filter")) return false;
        if(!get_param_error_tpl<bool>(nh, saturate_dtau_, "saturate_dtau")) return false;

        std::string robot_description;
        if(!get_param_error_tpl<std::string>(nh, robot_description, "/pinocchio_robot_description")) return false;

        model_pin_ = loadPandaPinocchio(robot_description);
        data_pin_ = pin::Data(model_pin_);

        if ((model_pin_.nq != 7) || (model_pin_.name != "panda"))
        {
            ROS_ERROR_STREAM("Problem when loading the robot urdf");
            return false;
        }

        ///////////////////
        // Claim interfaces
        ///////////////////
        // Retrieve resource FrankaStateHandle
        auto *franka_state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
        if (franka_state_interface == nullptr)
        {
            ROS_ERROR("CtrlJointSpaceID: Could not get Franka state interface from hardware");
            return false;
        }
        try
        {
            franka_state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(franka_state_interface->getHandle(arm_id + "_robot"));
        }
        catch (const hardware_interface::HardwareInterfaceException &ex)
        {
            ROS_ERROR_STREAM("CtrlJointSpaceID: Exception getting franka state handle: " << ex.what());
            return false;
        }

        // Retrieve resource FrankaModelHandle
        auto *model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
        if (model_interface == nullptr)
        {
            ROS_ERROR_STREAM("CtrlJointSpaceID: Error getting model interface from hardware");
            return false;
        }
        try
        {
            franka_model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(model_interface->getHandle(arm_id + "_model"));
        }
        catch (hardware_interface::HardwareInterfaceException &ex)
        {
            ROS_ERROR_STREAM("CtrlJointSpaceID: Exception getting model handle from interface: " << ex.what());
            return false;
        }

        // Retrieve resource FrankaModelHandle
        auto *effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
        if (effort_joint_interface == nullptr)
        {
            ROS_ERROR_STREAM("CtrlJointSpaceID: Error getting effort joint interface from hardware");
            return false;
        }
        for (size_t i = 0; i < 7; ++i)
        {
            try
            {
                joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
            }
            catch (const hardware_interface::HardwareInterfaceException &ex)
            {
                ROS_ERROR_STREAM("CtrlJointSpaceID: Exception getting joint handles: " << ex.what());
                return false;
            }
        }

        // Logs publishers
        configurations_publisher_.init(nh, "joint_configurations_comparison", 1);
        velocities_publisher_.init(nh, "joint_velocities_comparison", 1);
        torques_publisher_.init(nh, "joint_torques_comparison", 1);

        dq_filtered_ = Vector7d::Zero();

        return true;
    }

    void CtrlJointSpaceID::starting(const ros::Time &t0)
    {
        ROS_INFO_STREAM("CtrlJointSpaceID::starting");
        t_init_ = t0;
        q_init_ = Eigen::Map<const Vector7d>(franka_state_handle_->getRobotState().q.data());
    }

    void CtrlJointSpaceID::update(const ros::Time &t, const ros::Duration &period)
    {
        // ROS_INFO_STREAM("CtrlJointSpaceID::update t: " << t);

        // Time since start of the controller
        double Dt = (t - t_init_).toSec();

        // compute desired configuration and configuration velocity
        Vector7d q_r, dq_r, ddq_r;
        compute_sinusoid_joint_reference(delta_q_, period_q_, q_init_, Dt, q_r, dq_r, ddq_r);

        // Retrieve current measured robot state
        franka::RobotState robot_state = franka_state_handle_->getRobotState(); // return a const& of RobotState object -> not going to be modified
        Eigen::Map<Vector7d> q_m(robot_state.q.data());
        Eigen::Map<Vector7d> dq_m(robot_state.dq.data());
        Eigen::Map<Vector7d> tau_m(robot_state.tau_J.data());     // measured torques -> naturally contains gravity torque
        Eigen::Map<Vector7d> tau_J_d(robot_state.tau_J_d.data()); // desired torques (sent at previous iteration)

        // Filter the joint velocity measurements
        dq_filtered_ = (1 - alpha_dq_filter_) * dq_filtered_ + alpha_dq_filter_ * dq_m;

        // Compute desired torque
        Vector7d tau_d = compute_desired_torque(q_m, dq_m, dq_filtered_, q_r, dq_r, ddq_r, control_variant_, use_pinocchio_);

        // Compare torques sent at previous iteration with current desired torques, saturate if needed
        Vector7d tau_d_sat = saturate_dtau_ ? saturateTorqueRate(tau_d, tau_J_d, kDeltaTauMax_) : tau_d;

        // Send Torque Command
        for (size_t i = 0; i < 7; ++i)
        {
            joint_handles_[i].setCommand(tau_d_sat[i]);
        }

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

            Vector7d q_error = last_q_r_ - q_m;
            Vector7d dq_error = last_dq_r_ - dq_m;
            Vector7d tau_error = last_tau_d_ - tau_m;

            for (size_t i = 0; i < 7; ++i)
            {
                // Joint config
                configurations_publisher_.msg_.commanded[i] = last_q_r_[i];
                configurations_publisher_.msg_.measured[i] = q_m[i];
                configurations_publisher_.msg_.error[i] = q_error[i];

                // Joint velocities
                velocities_publisher_.msg_.commanded[i] = last_dq_r_[i];
                velocities_publisher_.msg_.measured[i] = dq_m[i];
                velocities_publisher_.msg_.error[i] = dq_error[i];

                // Joint torque
                torques_publisher_.msg_.commanded[i] = last_tau_d_[i];
                torques_publisher_.msg_.measured[i] = tau_m[i];
                torques_publisher_.msg_.error[i] = tau_error[i];
            }

            configurations_publisher_.unlockAndPublish();
            velocities_publisher_.unlockAndPublish();
            torques_publisher_.unlockAndPublish();
        }

        // Store previous desired/reference values
        last_q_r_ = q_r;
        last_dq_r_ = dq_r;
        last_tau_d_ = tau_d_sat + Eigen::Map<Vector7d>(franka_model_handle_->getGravity().data());
    }

    Vector7d CtrlJointSpaceID::compute_desired_torque(
        const Vector7d &q_m, const Vector7d &dq_m, const Vector7d &dq_filtered,
        const Vector7d &q_r, const Vector7d &dq_r, const Vector7d &ddq_r,
        JSIDVariant control_variant, bool use_pinocchio)
    {

        /**
         * Lagrangian dynamics
         *
         * M(q)*ddq + C(q,dq)*dq + g(q) = tau + J^T*f
         *
         * Here, we assume no contact so f=0.
         */

        // libfranka dynamics
        std::array<double, 7> coriolis_fra_arr = franka_model_handle_->getCoriolis();
        std::array<double, 7> gravity_fra_arr = franka_model_handle_->getGravity();
        std::array<double, 49> M_fra_arr = franka_model_handle_->getMass();
        // Eigen and Franka use Column-Major storage order (see model_pinocchio_vs_frank_controller)
        Eigen::Map<Vector7d> coriolis_fra(coriolis_fra_arr.data()); // C(q,dq)*dq
        Eigen::Map<Vector7d> gravity_fra(gravity_fra_arr.data());   // g(q)
        Eigen::Map<Matrix7d> M_fra(M_fra_arr.data());               // M(q)

        // Task error (joint trajectory error) and derivative, common to all variants
        Vector7d e = q_m - q_r;
        Vector7d de = dq_filtered - dq_r;

        Vector7d tau_d;
        switch (control_variant)
        {
        case JSIDVariant::IDControl:
        { // create local context to be able to init variables

            // ROS_INFO_STREAM("JSIDVariant::IDControl, pinocchio: " << use_pinocchio_);
            /**
             *
             * ddq_d = ddq_r - Kp * e - kd_arr * de
             * tau_cmd = M * ddq_d + h(q_m, dq_m)
             *         = rnea(q_m, dq_m, ddq_d)
             *
             * Use gain arrays to way the gains according to the inertia the
             *    -> need higher gains at the root of the robot
             * The best way to compute these relative scalings is to use the mass matrix like in IDControl
             * unless the model is not accurate enough
             */

            // Acceleration level feedback law
            Vector7d ddq_d = ddq_r - Kp_ * e - Kd_ * de;

            if (use_pinocchio_)
            {
                pin::rnea(model_pin_, data_pin_, q_m, dq_m, ddq_d);         // data.tau (but not data.g), joint torques
                pin::computeGeneralizedGravity(model_pin_, data_pin_, q_m); // data.g == generalized gravity

                // For pinocchio, substract the gravity term from rnea torque to get only centrifugal + Coriolis
                // For pinocchio, feedback is taken into account in rnea computation
                tau_d = data_pin_.tau - data_pin_.g;
            }
            else
                tau_d = M_fra * ddq_d + coriolis_fra;

            break;
        }
        case JSIDVariant::IDControlSimplified:
        { // create local context to be able to init variables

            // ROS_INFO_STREAM("JSIDVariant::IDControlSimplified, pinocchio: " << use_pinocchio_);
            /**
             * Decouples each joint level gain applied from the mass matrix: need an array of gains
             * but may be more robust to mass matrix model errors.
             *
             * tau_cmd = M*ddq_r + h(q_m, dq_m) - kp_arr * e - kd_arr * de
             *         = rnea(q_m, dq_m, ddq_r) - kp_arr * e - kd_arr * de
             *
             * Use gain arrays to way the gains according to the inertia the
             *    -> need higher gains at the root of the robot
             * The best way to compute these relative scalings is to use the mass matrix like in IDControl
             * unless the model is not accurate enough
             */

            pin::rnea(model_pin_, data_pin_, q_m, dq_m, ddq_r);         // data.tau (but not data.g), joint torques
            pin::computeGeneralizedGravity(model_pin_, data_pin_, q_m); // data.g == generalized gravity

            // For pinocchio, substract the gravity term from rnea torque to get only centrifugal + Coriolis
            Vector7d tau_feedback = -kp_gains_.cwiseProduct(e) - kd_gains_.cwiseProduct(de);
            if (use_pinocchio_)
                tau_d = data_pin_.tau - data_pin_.g + tau_feedback;
            else
                tau_d = M_fra * ddq_r + coriolis_fra + tau_feedback;

            break;
        }
        case JSIDVariant::PDGravity:
            // ROS_INFO_STREAM("JSIDVariant::PDGravity, pinocchio: " << use_pinocchio_);
            /**
             * tau_cmd = (g(q)) - kp_arr * e - kd_arr * de
             *
             * Rely on the gravity compensation implemented in Panda + feedback torque
             */
            tau_d = -kp_gains_.cwiseProduct(e) - kd_gains_.cwiseProduct(de);

            break;
        case JSIDVariant::PureGravity:
            // ROS_INFO_STREAM("JSIDVariant::PureGravity, pinocchio: " << use_pinocchio_);
            /**
             * tau_cmd = g(q)
             *
             * Rely purely on the gravity compensation implemented in Panda -> might be UNSTABLE
             */
            tau_d = Vector7d::Zero();

            break;
        default:
            ROS_INFO_STREAM("JSIDVariant " << control_variant << " not implemented !!!!");
        }

        return tau_d;
    }

    void CtrlJointSpaceID::compute_sinusoid_joint_reference(const Vector7d &delta_q, const Vector7d &period_q, const Vector7d &q0, double t,
                                                                  Vector7d &q_r, Vector7d &dq_r, Vector7d &ddq_r)
    {
        // a and c coeff vectors obtained for each joint using constraints:
        // q(t=0.0) = q0
        // q(t=period/2) = q0 + delta_q

        Vector7d w = (2 * M_PI / period_q.array()).matrix();
        Vector7d a = -delta_q;
        Vector7d c = q0 + delta_q;

        q_r = (a.array() * cos(w.array() * t)).matrix() + c;
        dq_r = (-w.array() * a.array() * sin(w.array() * t)).matrix();
        ddq_r = (-w.array().square() * a.array() * cos(w.array() * t)).matrix(); // non null initial acceleration!! needs to be dampened (e.g. torque staturation)
    }

    void CtrlJointSpaceID::stopping(const ros::Time &t0)
    {
        ROS_INFO_STREAM("CtrlJointSpaceID::stopping");
    }

} // namespace panda_torque_mpc

PLUGINLIB_EXPORT_CLASS(panda_torque_mpc::CtrlJointSpaceID,
                       controller_interface::ControllerBase)
