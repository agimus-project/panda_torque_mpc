// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
// Adapted from panda_torque_mpc joint_impedance_example_controller

#pragma once

#include <memory>
#include <string>
#include <vector>
#include <math.h>
#include <fstream>


#include <pinocchio/fwd.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/trigger_rate.h>

#include "panda_torque_mpc/JointValuesComparison.h"

#include "panda_torque_mpc/common.h"

namespace panda_torque_mpc
{

    namespace pin = pinocchio;

    class CtrlPlaybackPDplus : public controller_interface::MultiInterfaceController<franka_hw::FrankaModelInterface,
                                                                                     franka_hw::FrankaStateInterface,
                                                                                     hardware_interface::EffortJointInterface>
    {

    public:
        bool init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle) override;
        void starting(const ros::Time &) override;
        void update(const ros::Time &, const ros::Duration &period) override;
        void stopping(const ros::Time &) override;

    private:
        // Handles
        std::unique_ptr<franka_hw::FrankaModelHandle> franka_model_handle_;
        std::unique_ptr<franka_hw::FrankaStateHandle> franka_state_handle_;
        std::vector<hardware_interface::JointHandle> joint_handles_;

        // Torque saturation
        const double kDeltaTauMax_{1.0}; // using static constexpr creates an undefined symbol error

        // Controller parameters
        double scale_ff_;              // scale down feedforward torque
        double Kp_, Kd_;               // PD gains
        double alpha_dq_filter_;
        bool saturate_dtau_;
        

        // Publishers
        franka_hw::TriggerRate rate_trigger_{1.0};
        realtime_tools::RealtimePublisher<JointValuesComparison> configurations_publisher_;
        realtime_tools::RealtimePublisher<JointValuesComparison> velocities_publisher_;
        realtime_tools::RealtimePublisher<JointValuesComparison> torques_publisher_;

        // Pinocchio objects
        pin::Model model_pin_;
        pin::Data data_pin_;

        // input file streams and file path to read joint trajectory files
        std::ifstream fs_q_;
        std::ifstream fs_v_;
        std::ifstream fs_tau_;
        std::string traj_dir_;
        std::vector<Vector7d> q_vec_;
        std::vector<Vector7d> v_vec_;
        std::vector<Vector7d> tau_vec_;
        unsigned int i_line_;
        unsigned int nb_lines_;
        Vector7d q_r_;
        Vector7d dq_r_;
        Vector7d tau_ff_;

        // other vars 
        Vector7d dq_filtered_;
        Vector7d last_q_r_;
        Vector7d last_dq_r_;
        Vector7d last_tau_d_;

    };

} // namespace panda_torque_mpc
