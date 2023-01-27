// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
// Adapted from panda_torque_mpc joint_impedance_example_controller 

#pragma once

#include <memory>
#include <string>
#include <vector>
#include <math.h>


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

#include "panda_torque_mpc/JointConfigurationComparison.h"
#include "panda_torque_mpc/JointVelocityComparison.h"
#include "panda_torque_mpc/JointTorqueComparison.h"

#include "panda_torque_mpc/common.h"


namespace panda_torque_mpc {



namespace pin = pinocchio;

class JointSpaceIDController : 
    public controller_interface::MultiInterfaceController<franka_hw::FrankaModelInterface,
                                                          franka_hw::FrankaStateInterface,
                                                          hardware_interface::EffortJointInterface> {

 public:
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;
  void stopping(const ros::Time&) override;

  enum Variant {
    IDControl,
    IDControlSimplified,
    PDGravity
  };

 private:
  // Handles  
  std::unique_ptr<franka_hw::FrankaModelHandle> franka_model_handle_;
  std::unique_ptr<franka_hw::FrankaStateHandle> franka_state_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;
  
  // Torque saturation
  std::array<double, 7> saturateTorqueRate(
      const std::array<double, 7>& tau_d_calculated,
      const std::array<double, 7>& tau_J_d);  // NOLINT (readability-identifier-naming)
  const double kDeltaTauMax{1.0};  // using static constexpr creates an undefined symbol error

  // Controller parameters
  std::vector<double> k_gains_;
  std::vector<double> d_gains_;
  std::vector<double> delta_q_;
  std::vector<double> period_q_;
  bool use_pinocchio_;
  double alpha_dq_filter_;
  Variant control_variant_;

  // Current update state
  std::array<double, 7> last_q_r_;
  std::array<double, 7> last_dq_r_;
  std::array<double, 7> dq_filtered_;

  // initial values
  ros::Time t_init_;
  std::array<double, 7> q_init_;   

  franka_hw::TriggerRate rate_trigger_{1.0};
  std::array<double, 7> last_tau_d_{};
  realtime_tools::RealtimePublisher<JointConfigurationComparison> configurations_publisher_;
  realtime_tools::RealtimePublisher<JointVelocityComparison> velocities_publisher_;
  realtime_tools::RealtimePublisher<JointTorqueComparison> torques_publisher_;

  // Pinocchio objects
  pin::Model model_pin_;
  pin::Data data_pin_;

  void compute_sinusoid_joint_reference(const std::vector<double>& delta_q, const std::vector<double>& period_q, const std::array<double, 7>& q0, double t,
                                        std::array<double, 7>& q_r, std::array<double, 7>& dq_r, std::array<double, 7>& ddq_r);

};

} // namespace panda_torque_mpc
