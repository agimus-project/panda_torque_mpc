#include <panda_torque_mpc/ctrl_playback_pd_plus.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka/robot_state.h>




namespace panda_torque_mpc
{


    bool get_vector_from_csv_fs_line(std::ifstream& _fs, Vector7d& _vec)
    {
            std::string line;
            // read current line, if we are already the end of the file
            if (!std::getline(_fs, line)) return false;
            std::stringstream ss(line);

            // go through the "v1,v2,v3...,v7" line extracting each vi delimited by ","
            std::string value_str;
            std::vector<double> val_vec;
            while (std::getline(ss, value_str, ','))
            {
                // !!! stod likely to fail due to \n character
                val_vec.push_back(std::stod(value_str));
            }
            // Check vector size
            if (val_vec.size() != 7) return false;
            _vec = Eigen::Map<Vector7d>(val_vec.data());
            
            return true;
    }

    bool CtrlPlaybackPDplus::init(hardware_interface::RobotHW *robot_hw,
                                      ros::NodeHandle &node_handle)
    {

        ///////////////////
        // Load parameters
        ///////////////////
        std::string arm_id;
        if (!node_handle.getParam("arm_id", arm_id))
        {
            ROS_ERROR("CtrlPlaybackPDplus: Could not read parameter arm_id");
            return false;
        }

        std::vector<std::string> joint_names;
        if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7)
        {
            ROS_ERROR("CtrlPlaybackPDplus: Invalid or no joint_names parameters provided, aborting controller init!");
            return false;
        }


        if (!node_handle.getParam("traj_dir", traj_dir_))
        {
            ROS_ERROR("CtrlPlaybackPDplus: Could not read parameter traj_dir");
            return false;
        }

        if (!node_handle.getParam("scale_ff", scale_ff_))
        {
            ROS_ERROR("CtrlPlaybackPDplus: Could not read parameter scale_ff");
            return false;
        }

        if (!node_handle.getParam("Kp", Kp_))
        {
            ROS_ERROR("CtrlPlaybackPDplus: Could not read parameter Kp");
            return false;
        }

        if (!node_handle.getParam("Kd", Kd_))
        {
            ROS_ERROR("CtrlPlaybackPDplus: Could not read parameter Kd");
            return false;
        }

        double publish_rate(30.0);
        if (!node_handle.getParam("publish_rate", publish_rate))
        {
            ROS_INFO_STREAM("CtrlPlaybackPDplus: publish_rate not found. Defaulting to " << publish_rate);
        }
        rate_trigger_ = franka_hw::TriggerRate(publish_rate);

        if (!node_handle.getParam("alpha_dq_filter", alpha_dq_filter_))
        {
            ROS_ERROR_STREAM("CtrlPlaybackPDplus: Could not read parameter alpha_dq_filter");
        }

        if (!node_handle.getParam("saturate_dtau", saturate_dtau_))
        {
            ROS_ERROR_STREAM("CtrlPlaybackPDplus: Could not read parameter saturate_dtau");
        }

        ///////////////////
        // Claim interfaces
        ///////////////////
        // Retrieve resource FrankaStateHandle
        auto *franka_state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
        if (franka_state_interface == nullptr)
        {
            ROS_ERROR("CtrlPlaybackPDplus: Could not get Franka state interface from hardware");
            return false;
        }
        try
        {
            franka_state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(franka_state_interface->getHandle(arm_id + "_robot"));
        }
        catch (const hardware_interface::HardwareInterfaceException &ex)
        {
            ROS_ERROR_STREAM("CtrlPlaybackPDplus: Exception getting franka state handle: " << ex.what());
            return false;
        }

        // Retrieve resource FrankaModelHandle
        auto *model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
        if (model_interface == nullptr)
        {
            ROS_ERROR_STREAM("CtrlPlaybackPDplus: Error getting model interface from hardware");
            return false;
        }
        try
        {
            franka_model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(model_interface->getHandle(arm_id + "_model"));
        }
        catch (hardware_interface::HardwareInterfaceException &ex)
        {
            ROS_ERROR_STREAM("CtrlPlaybackPDplus: Exception getting model handle from interface: " << ex.what());
            return false;
        }

        // Retrieve resource FrankaModelHandle
        auto *effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
        if (effort_joint_interface == nullptr)
        {
            ROS_ERROR_STREAM("CtrlPlaybackPDplus: Error getting effort joint interface from hardware");
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
                ROS_ERROR_STREAM("CtrlPlaybackPDplus: Exception getting joint handles: " << ex.what());
                return false;
            }
        }

        // Open trajectory files
        std::string path_q = traj_dir_ + "q.csv";
        std::string path_v = traj_dir_ + "v.csv";
        std::string path_tau = traj_dir_ + "tau.csv";
        std::cout << "Playback files:" << std::endl;
        std::cout << path_q << std::endl;
        std::cout << path_v << std::endl;
        std::cout << path_tau << std::endl;
        fs_q_.open(path_q);
        fs_v_.open(path_v);
        fs_tau_.open(path_tau);
        if (!fs_q_.is_open()){
            ROS_ERROR_STREAM("path_q: failed to open csv file " << path_q);
            return false;
        }
        if (!fs_v_.is_open()){
            ROS_ERROR_STREAM("path_v: failed to open csv file " << path_v);
            return false;
        }
        if (!fs_tau_.is_open()){
            ROS_ERROR_STREAM("path_tau: failed to open csv file " << path_tau);
            return false;
        }

        // Store in advance all the lines in vectors to avoid reading the filestream at update freq (it was tested and was failing for unclear reasons)
        while (get_vector_from_csv_fs_line(fs_q_,   q_r_) &&
               get_vector_from_csv_fs_line(fs_v_,   dq_r_) &&
               get_vector_from_csv_fs_line(fs_tau_, tau_ff_))
        {
            q_vec_.push_back(q_r_);
            v_vec_.push_back(dq_r_);
            tau_vec_.push_back(tau_ff_);
        }

        i_line_ = 0;
        nb_lines_ = q_vec_.size();

        fs_q_.close();
        fs_v_.close();
        fs_tau_.close();

        // ROS publishers
        configurations_publisher_.init(node_handle, "joint_configurations_comparison", 1);
        velocities_publisher_.init(node_handle, "joint_velocities_comparison", 1);
        torques_publisher_.init(node_handle, "joint_torques_comparison", 1);

        dq_filtered_ = Vector7d::Zero();

        return true;
    }

    void CtrlPlaybackPDplus::starting(const ros::Time &t0)
    {
        ROS_INFO_STREAM("CtrlPlaybackPDplus::starting");
    }

    void CtrlPlaybackPDplus::update(const ros::Time &t, const ros::Duration &period)
    {

        // Retrieve current measured robot state
        franka::RobotState robot_state = franka_state_handle_->getRobotState(); // return a const& of RobotState object -> not going to be modified
        Eigen::Map<Vector7d> q_m(robot_state.q.data());
        Eigen::Map<Vector7d> dq_m(robot_state.dq.data());
        Eigen::Map<Vector7d> tau_m(robot_state.tau_J.data());     // measured torques -> naturally contains gravity torque
        Eigen::Map<Vector7d> tau_J_d(robot_state.tau_J_d.data()); // desired torques (sent at previous iteration)

        // Filter the joint velocity measurements
        dq_filtered_ = (1 - alpha_dq_filter_) * dq_filtered_ + alpha_dq_filter_ * dq_m;

        // retrieve the current trajectory point
        // Assumes that no update jump was done
        if (i_line_ < nb_lines_)
        {
            q_r_ = q_vec_[i_line_];
            dq_r_ = v_vec_[i_line_];
            tau_ff_ = tau_vec_[i_line_];
            i_line_++;
        }

        // Compute PD+ torque
        Vector7d tau_d = scale_ff_*tau_ff_ - Kp_*(q_m - q_r_) - Kd_*(dq_m - dq_r_);

        // Remove gravity from commanded torque (assuming it was provided by tau_ff)
        Eigen::Map<Vector7d> tau_gravity(franka_model_handle_->getGravity().data());
        tau_d -= tau_gravity;

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
        last_q_r_ = q_r_;
        last_dq_r_ = dq_r_;
        last_tau_d_ = tau_d_sat + Eigen::Map<Vector7d>(franka_model_handle_->getGravity().data());
    }

    void CtrlPlaybackPDplus::stopping(const ros::Time &t0)
    {
        ROS_INFO_STREAM("CtrlPlaybackPDplus::stopping");
    }

} // namespace panda_torque_mpc

PLUGINLIB_EXPORT_CLASS(panda_torque_mpc::CtrlPlaybackPDplus,
                       controller_interface::ControllerBase)
