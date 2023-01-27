// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <panda_torque_mpc/pdp_joint_tracking_controller.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka/robot_state.h>

namespace panda_torque_mpc {

bool PDPJointTrackingController::init(hardware_interface::RobotHW* robot_hw,
                                           ros::NodeHandle& node_handle) {

  ///////////////////
  // Load parameters
  ///////////////////
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("PDPJointTrackingController: Could not read parameter arm_id");
    return false;
  }

  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR("PDPJointTrackingController: Invalid or no joint_names parameters provided, aborting controller init!");
    return false;
  }

  if (!node_handle.getParam("k_gains", k_gains_) || k_gains_.size() != 7) {
    ROS_ERROR("PDPJointTrackingController:  Invalid or no k_gains parameters provided, aborting controller init!");
    return false;
  }

    if (!node_handle.getParam("d_gains", d_gains_) || k_gains_.size() != 7) {
    ROS_ERROR("PDPJointTrackingController:  Invalid or no d_gains parameters provided, aborting controller init!");
    return false;
  }

  if (!node_handle.getParam("delta_q", delta_q_) || delta_q_.size() != 7) {
    ROS_ERROR("PDPJointTrackingController:  Invalid or no delta_q parameters provided, aborting controller init!");
    return false;
  }

  if (!node_handle.getParam("period_q", period_q_) || period_q_.size() != 7) {
    ROS_ERROR("PDPJointTrackingController:  Invalid or no period_q parameters provided, aborting controller init!");
    return false;
  }

  double publish_rate(30.0);
  if (!node_handle.getParam("publish_rate", publish_rate)) {
    ROS_INFO_STREAM("PDPJointTrackingController: publish_rate not found. Defaulting to " << publish_rate);
  }
  rate_trigger_ = franka_hw::TriggerRate(publish_rate);

  if (!node_handle.getParam("use_pinocchio", use_pinocchio_)) {
    ROS_ERROR_STREAM("PDPJointTrackingController: use_pinocchio not found. Defaulting to " << use_pinocchio_);
  }

  if (!node_handle.getParam("alpha_dq_filter", alpha_dq_filter_)) {
    ROS_ERROR_STREAM("PDPJointTrackingController: alpha_dq_filter not found. Defaulting to " << alpha_dq_filter_);
  }

  // Load panda model with pinocchio
  std::string urdf_path;
  if (!node_handle.getParam("urdf_path", urdf_path)) {
    ROS_ERROR("PDPJointTrackingController: Could not read parameter urdf_path");
    return false;
  }
  pin::urdf::buildModel(urdf_path, model_pin_);
  std::cout << "model name: " << model_pin_.name << std::endl;
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
  auto* franka_state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (franka_state_interface == nullptr) {
    ROS_ERROR("ModelPinocchioVsFrankaController: Could not get Franka state interface from hardware");
    return false;
  }
  try {
    franka_state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(franka_state_interface->getHandle(arm_id + "_robot"));
  } catch (const hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM("ModelPinocchioVsFrankaController: Exception getting franka state handle: " << ex.what());
    return false;
  }

  // Retrieve resource FrankaModelHandle
  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM("ModelPinocchioVsFrankaController: Error getting model interface from hardware");
    return false;
  }
  try {
    franka_model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM("ModelPinocchioVsFrankaController: Exception getting model handle from interface: " << ex.what());
    return false;
  }

  // Retrieve resource FrankaModelHandle
  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM("PDPJointTrackingController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM("PDPJointTrackingController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  torques_publisher_.init(node_handle, "torque_comparison", 1);

  std::fill(dq_filtered_.begin(), dq_filtered_.end(), 0);

  return true;
}

void PDPJointTrackingController::starting(const ros::Time& t0) {
  ROS_INFO_STREAM("PDPJointTrackingController::starting");
  t_init_ = t0;
  q_init_ = franka_state_handle_->getRobotState().q;
}

void PDPJointTrackingController::update(const ros::Time& t,
                                        const ros::Duration& period) {

  // Time since start of the controller
  double Dt = (t - t_init_).toSec();

  // compute desired configuration and configuration velocity
  std::array<double, 7> q_d;
  std::array<double, 7> dq_d;
  for (size_t i = 0; i < 7; i++) {
    double omg = 2*M_PI/period_q_[i];
    double A = - 0.5 * delta_q_[i];
    double C = q_init_[i] + 0.5 * delta_q_[i];

    q_d[i]  =      A*cos(omg*Dt) + C;
    dq_d[i] = -omg*A*sin(omg*Dt);
  }

  // Retrieve current robot state
  franka_state_handle_
  franka::RobotState robot_state = franka_state_handle_->getRobotState();
  q_arr_ = robot_state.q;
  dq_arr_ = robot_state.dq;
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q_pin(q_arr_.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq_pin(dq_arr_.data());
  Eigen::Matrix<double, 7, 1> ddq_pin = Eigen::Matrix<double, 7, 1>::Zero();  // let's neglect the inertia terms for now

  for (size_t i = 0; i < 7; i++) {
    dq_filtered_[i] = (1 - alpha_dq_filter_) * dq_filtered_[i] + alpha_dq_filter_ * dq_arr_[i];
  }

  // pinocchio dynamics
  pin::forwardKinematics(model_pin_, data_pin_, q_pin, dq_pin);  // joint frame placements
  pin::rnea(model_pin_, data_pin_, q_pin, dq_pin, ddq_pin);      // data.tau (but not data.g)
  pin::computeGeneralizedGravity(model_pin_, data_pin_, q_pin);  // data.g

  // libfranka dynamics
  std::array<double, 7> coriolis_fra = franka_model_handle_->getCoriolis();
  std::array<double, 7> gravity_fra = franka_model_handle_->getGravity();
  

  // PD+ computation
  std::array<double, 7> tau_d_calculated;
  double tau_ff = 0.0;  // should NOT include gravity terms since taken care of by internal Franka controller
  for (size_t i = 0; i < 7; ++i) {
    if (use_pinocchio_){
      // substract the gravity term from rnea torque to get only centrifugal + Coriolis 
      tau_ff = data_pin_.tau[i] - data_pin_.g[i];
    }
    else {
      tau_ff = coriolis_fra[i];
    }
    tau_d_calculated[i] = tau_ff +
                          k_gains_[i] * (q_d[i]  - q_arr_[i]) +
                          d_gains_[i] * (dq_d[i] - dq_filtered_[i]);
  }

  // Maximum torque difference with a sampling rate of 1 kHz. The maximum torque rate is
  // 1000 * (1 / sampling_time).
  std::array<double, 7> tau_d_saturated = saturateTorqueRate(tau_d_calculated, robot_state.tau_J_d);

  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d_saturated[i]);
  }


  // Publish logs
  if (rate_trigger_() && torques_publisher_.trylock()) {
    std::array<double, 7> tau_j = robot_state.tau_J;
    std::array<double, 7> tau_error;
    double error_rms(0.0);
    for (size_t i = 0; i < 7; ++i) {
      tau_error[i] = last_tau_d_[i] - tau_j[i];
      error_rms += std::sqrt(std::pow(tau_error[i], 2.0)) / 7.0;
    }
    torques_publisher_.msg_.root_mean_square_error = error_rms;
    for (size_t i = 0; i < 7; ++i) {
      torques_publisher_.msg_.tau_commanded[i] = last_tau_d_[i];
      torques_publisher_.msg_.tau_error[i] = tau_error[i];
      torques_publisher_.msg_.tau_measured[i] = tau_j[i];
    }
    // ROS_INFO_STREAM("last_tau_j[1]: " << last_tau_d_[1]);
    // ROS_INFO_STREAM("tau_j[1]     : " << tau_j[1]);
    torques_publisher_.unlockAndPublish();
  }

  for (size_t i = 0; i < 7; ++i) {
    // last_tau_d_[i] = tau_d_saturated[i] + gravity_fra[i];
    last_tau_d_[i] = -(tau_d_saturated[i] + gravity_fra[i]);  // WHY is there a change of sign??
  }
}

std::array<double, 7> PDPJointTrackingController::saturateTorqueRate(
    const std::array<double, 7>& tau_d_calculated,
    const std::array<double, 7>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  std::array<double, 7> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, kDeltaTauMax), -kDeltaTauMax);
  }
  return tau_d_saturated;
}


void PDPJointTrackingController::stopping(const ros::Time& t0) {
  ROS_INFO_STREAM("PDPJointTrackingController::stopping");
}

}  // namespace panda_torque_mpc

PLUGINLIB_EXPORT_CLASS(panda_torque_mpc::PDPJointTrackingController,
                       controller_interface::ControllerBase)
