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

class TaskSpaceIDController : 
    public controller_interface::MultiInterfaceController<franka_hw::FrankaModelInterface,
                                                          franka_hw::FrankaStateInterface,
                                                          hardware_interface::EffortJointInterface> {

 public:
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;
  void stopping(const ros::Time&) override;

 private:
  // Handles  
  std::unique_ptr<franka_hw::FrankaModelHandle> franka_model_handle_;
  std::unique_ptr<franka_hw::FrankaStateHandle> franka_state_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;
  
  // Torque saturation
  Vector7d saturateTorqueRate(
      const Vector7d& tau_d_calculated,
      const Vector7d& tau_J_d);  // NOLINT (readability-identifier-naming)
  const double kDeltaTauMax_{1.0};  // using static constexpr creates an undefined symbol error

  // Controller parameters
  double Kp_, Kd_;                // task space gains Control
  Vector6d delta_nu_, period_nu_;   // trajectory specification
  bool use_pinocchio_;
  double alpha_dq_filter_;

  // Current update state
  Vector7d last_q_r_;
  Vector7d last_dq_r_;
  Vector7d last_tau_d_{};
  Vector7d dq_filtered_;

  // initial values
  ros::Time t_init_;
  Vector7d q_init_;
  pin::SE3 x_init_;

  // Publishers
  franka_hw::TriggerRate rate_trigger_{1.0};
  realtime_tools::RealtimePublisher<JointValuesComparison> configurations_publisher_;
  realtime_tools::RealtimePublisher<JointValuesComparison> velocities_publisher_;
  realtime_tools::RealtimePublisher<JointValuesComparison> torques_publisher_;

  // Pinocchio objects
  pin::Model model_pin_;
  pin::Data data_pin_;

  // other 
  franka::Frame franka_frame_;
  std::string pin_frame_;

  /**
   * \brief Compute torque required to achieve end effector pose trajectory tracking.
   * 
   * @param[in] q_m measured joint configuration 
   * @param[in] dq_m measured joint velocity 
   * @param[in] dq_filtered filtered joint velocity 
   * @param[in] x_r target end effector pose
   * @param[in] dx_r target end effector spatial velocity
   * @param[in] ddx_r target end effector spatial acceleration 
   * @param[in] use_pinocchio use pinocchio for Rigid Body Dynamics Algorithms if true (else libfranka) 
  */
  Vector7d compute_desired_torque(
      const Vector7d& q_m, const Vector7d& dq_m, const Vector7d& dq_filtered, 
      const pin::SE3& x_r, const pin::Motion& dx_r, const pin::Motion& ddx_r, 
      bool use_pinocchio);

  /**
   * \brief Generate a (cos)sinusoidal target trajectory of end effector pose.
   * 
   * @param[in] delta_nu trajectory parameter: vector (size 6, [lin,rot]) of the delta motion amplitude (on the SE3 local tangent space)   
   * @param[in] period_nu trajectory parameter: vector (size 6, [lin,rot]) of period for delta motion axes (on the SE3 local tangent space)
   * @param[in] pose_0 initial pose  
   * @param[in] t current time with q(0) = q0
   * @param[out] x_r target joint configuration
   * @param[out] dx_r target joint velocity
   * @param[out] ddx_r target joint acceleration 
  */
  void compute_sinusoid_pose_reference(const Vector6d& delta_nu, const Vector6d& period_nu, const pin::SE3& pose_0, double t,
                                       pin::SE3& x_r, pin::Motion& dx_r, pin::Motion& ddx_r);

};

} // namespace panda_torque_mpc
