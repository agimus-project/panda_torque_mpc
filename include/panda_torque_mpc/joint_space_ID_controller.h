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

#include "panda_torque_mpc/JointValuesComparison.h"

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

  enum JSIDVariant {
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
  Vector7d saturateTorqueRate(
      const Vector7d& tau_d_calculated,
      const Vector7d& tau_J_d);  // NOLINT (readability-identifier-naming)
  const double kDeltaTauMax{1.0};  // using static constexpr creates an undefined symbol error

  // Controller parameters
  double Kp_, Kd_;                // IDControl
  Vector7d kp_gains_, kd_gains_;  // IDControlSimplified
  Vector7d delta_q_, period_q_;   // trajectory specification
  JSIDVariant control_variant_;
  bool use_pinocchio_;
  double alpha_dq_filter_;

  // Current update state
  Vector7d last_q_r_;
  Vector7d last_dq_r_;
  Vector7d dq_filtered_;

  // initial values
  ros::Time t_init_;
  Vector7d q_init_;   

  franka_hw::TriggerRate rate_trigger_{1.0};
  Vector7d last_tau_d_{};
  realtime_tools::RealtimePublisher<JointValuesComparison> configurations_publisher_;
  realtime_tools::RealtimePublisher<JointValuesComparison> velocities_publisher_;
  realtime_tools::RealtimePublisher<JointValuesComparison> torques_publisher_;

  // Pinocchio objects
  pin::Model model_pin_;
  pin::Data data_pin_;

  /**
   * \brief Compute torque required to achieved trajectory tracking.
   * 
   * Implement the different JSIDVariant which parameters are stored as class attributes.
   * 
   * @param[in] q_m: measured joint configuration 
   * @param[in] dq_m: measured joint velocity 
   * @param[in] dq_filtered: filtered joint velocity 
   * @param[in] q_r: target joint configuration
   * @param[in] dq_r: target joint velocity
   * @param[in] ddq_r: target joint acceleration 
   * @param[in] control_variant: selection of the type of controller 
   * @param[in] use_pinocchio: use pinocchio for Rigid Body Dynamics Algorithms if true (else libfranka) 
  */
  Vector7d compute_desired_torque(
      const Vector7d& q_m, const Vector7d& dq_m, const Vector7d& dq_filtered, 
      const Vector7d& q_r, const Vector7d& dq_r, const Vector7d& ddq_r, 
      JSIDVariant control_variant, bool use_pinocchio);

  /**
   * \brief Generate a (cos)sinusoidal target trajectory of joint configurations.
   * 
   * @param[in] delta_q: trajectory parameter: vector (size 7) of configuration difference for each joint   
   * @param[in] period_q: trajectory parameter: vector (size 7) of movement period for each joint   
   * @param[in] q0: initial configuration 
   * @param[in] t: current time with q(0) = q0
   * @param[out] q_r: target joint configuration
   * @param[out] dq_r: target joint velocity
   * @param[out] ddq_r: target joint acceleration 
  */
  void compute_sinusoid_joint_reference(const Vector7d& delta_q, const Vector7d& period_q, const Vector7d& q0, double t,
                                        Vector7d& q_r, Vector7d& dq_r, Vector7d& ddq_r);

};

} // namespace panda_torque_mpc
