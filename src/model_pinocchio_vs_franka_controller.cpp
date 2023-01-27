// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

#include "panda_torque_mpc/model_pinocchio_vs_franka_controller.h"

#include <algorithm>
#include <array>
#include <cstring>
#include <iterator>
#include <memory>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>



namespace {
template <class T, size_t N>
std::ostream& operator<<(std::ostream& ostream, const std::array<T, N>& array) {
  ostream << "[";
  std::copy(array.cbegin(), array.cend() - 1, std::ostream_iterator<T>(ostream, ","));
  std::copy(array.cend() - 1, array.cend(), std::ostream_iterator<T>(ostream));
  ostream << "]";
  return ostream;
}
}  // anonymous namespace

namespace panda_torque_mpc {

bool ModelPinocchioVsFrankaController::init(hardware_interface::RobotHW* robot_hw,
                                  ros::NodeHandle& node_handle) {

  ///////////////////
  // Load parameters
  ///////////////////
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("ModelPinocchioVsFrankaController: Could not read parameter arm_id");
    return false;
  }

  // Load Pinocchio urdf
  std::string urdf_path;
  if (!node_handle.getParam("urdf_path", urdf_path)) {
    ROS_ERROR("ModelPinocchioVsFrankaController: Could not read parameter urdf_path");
    return false;
  }
  // Load panda model with pinocchio
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

  return true;
}

void ModelPinocchioVsFrankaController::update(const ros::Time& /*time*/, const ros::Duration& /*period*/) {
  if (rate_trigger_()) { 
    // FRANKA MODEL
    // -Frank Rigid Body Dynamics computations are done on "Control" computer, libfranka only communicates
    // with it to retrieve data.
    // - What is the definition of franka::Frame::kEndEffector?
    std::array<double, 49> mass = franka_model_handle_->getMass();
    std::array<double, 7> gravity = franka_model_handle_->getGravity();
    std::array<double, 7> coriolis = franka_model_handle_->getCoriolis();
    std::array<double, 16> pose_j4 = franka_model_handle_->getPose(franka::Frame::kJoint4);
    std::array<double, 16> pose_j7 = franka_model_handle_->getPose(franka::Frame::kJoint7);
    // std::array<double, 16> pose_ee = franka_model_handle_->getPose(franka::Frame::kEndEffector);  
    std::array<double, 42> joint4_body_jacobian = franka_model_handle_->getBodyJacobian(franka::Frame::kJoint4);
    std::array<double, 42> joint7_zero_jacobian = franka_model_handle_->getZeroJacobian(franka::Frame::kJoint7);

    // Retrieve current robot state from state handle
    franka::RobotState robot_state = franka_state_handle_->getRobotState();
    Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
    Eigen::Matrix<double, 7, 1> ddq = Eigen::Matrix<double, 7, 1>::Zero();
    
    // pinocchio computations (stored in data_pin_)
    // M(q)*ddq + C(q,dq)*dq + g(q) = tau + J^T*fext
    pin::forwardKinematics(model_pin_, data_pin_, q, dq);      // joint frame placements
    pin::updateFramePlacements(model_pin_, data_pin_);         // link frame placements
    pin::computeCoriolisMatrix(model_pin_, data_pin_, q, dq);  // C(q, dq)
    pin::computeGeneralizedGravity(model_pin_, data_pin_, q);  // g(q)
    pin::crba(model_pin_, data_pin_, q);                       // M(q)
    pin::rnea(model_pin_, data_pin_, q, dq, ddq);              // data.tau

    
    // Pinocchio frame ids: joints are a subset of all frames, their ids correspond to the kinematic chain ordee.
    // Pinocchio joint ids: id in the collection of non-fixed joints. The list starts always with universe, so first real joint has id 1.
    auto jid_4 = model_pin_.getJointId("panda_joint4");
    auto jid_7 = model_pin_.getJointId("panda_joint7");
    auto fid_4 = model_pin_.getFrameId("panda_joint4");
    auto fid_7 = model_pin_.getFrameId("panda_joint7");

    ////////////////////////////////
    // Store everything in Eigen::Matrix objects for easy comparison
    // Eigen and Franka use Column-Major storage order

    // Pose as homogeneous transformation matrices 
    Eigen::Matrix<double, 4, 4> oM4_pin = data_pin_.oMi.at(jid_4).toHomogeneousMatrix();
    Eigen::Matrix<double, 4, 4> oM7_pin = data_pin_.oMi.at(jid_7).toHomogeneousMatrix();
    Eigen::Map<Eigen::Matrix<double, 4, 4>> oM4_fra(pose_j4.data()); 
    Eigen::Map<Eigen::Matrix<double, 4, 4>> oM7_fra(pose_j7.data()); 
    // Frame jacobians Ji st. vi = Ji * q
    // !! call setZero since pinocchio does not internally. Unitialized values appear for frames in the middle of the kinematic chain 
    Eigen::Matrix<double, 6, 7> lJ4_pin_f; lJ4_pin_f.setZero(); pin::computeFrameJacobian(model_pin_, data_pin_, q, fid_4, lJ4_pin_f);  
    Eigen::Matrix<double, 6, 7> lJ4_pin_j; lJ4_pin_j.setZero(); pin::computeJointJacobian(model_pin_, data_pin_, q, jid_4, lJ4_pin_j);  // no diff with computeFrameJacobian if corresponding joints AND computeFrameJacobian called with pin::LOCAL (default)
    Eigen::Matrix<double, 6, 7> oJ7_pin_f; oJ7_pin_f.setZero(); pin::computeFrameJacobian(model_pin_, data_pin_, q, fid_7, pin::WORLD, oJ7_pin_f); 
    Eigen::Matrix<double, 6, 7> loJ7_pin_f; loJ7_pin_f.setZero(); pin::computeFrameJacobian(model_pin_, data_pin_, q, fid_7, pin::LOCAL_WORLD_ALIGNED, loJ7_pin_f); 
    Eigen::Map<Eigen::Matrix<double, 6, 7>> lJ4_fra_j(joint4_body_jacobian.data()); 
    Eigen::Map<Eigen::Matrix<double, 6, 7>> oJ7_fra_j(joint7_zero_jacobian.data()); 
    // Lagrangian dynamics equation elements
    Eigen::Matrix<double, 7, 1> g_pin = data_pin_.g;
    Eigen::Matrix<double, 7, 7> C_pin = data_pin_.C;
    Eigen::Matrix<double, 7, 7> M_pin = data_pin_.M;
    Eigen::Map<Eigen::Matrix<double, 7, 1>> g_fra(gravity.data()); 
    Eigen::Map<Eigen::Matrix<double, 7, 1>> cor_fra(coriolis.data()); 
    Eigen::Map<Eigen::Matrix<double, 7, 7>> M_fra(mass.data()); 
    
    


    ROS_INFO("\n\n\n--------------------------------------------------");
    // ROS_INFO_STREAM("mass :" << mass);
    // ROS_INFO_STREAM("coriolis: " << coriolis);
    // ROS_INFO_STREAM("gravity :" << gravity);
    // ROS_INFO_STREAM("pose_j4_fra :" << pose_j4);
    // ROS_INFO_STREAM("joint4_body_jacobian :" << joint4_body_jacobian);
    // ROS_INFO_STREAM("joint_zero_jacobian :" << endeffector_zero_jacobian);


    // ROS_INFO_STREAM("\ndiff mass:\n" << mass);
    // ROS_INFO_STREAM("\ndiff coriolis: \n" << coriolis);
    ROS_INFO_STREAM("\ngravity_fra :\n" << (g_fra).transpose());
    ROS_INFO_STREAM("\ngravity_pin :\n" << (g_pin).transpose());
    ROS_INFO_STREAM("\ndiff gravity :\n" << (g_fra - g_pin).transpose());
    // ROS_INFO_STREAM("\ngravity fra:\n" << g_fra.transpose());
    // ROS_INFO_STREAM("\ngravity pin:\n" << g_pin.transpose());
    ROS_INFO_STREAM("\ndiff data_pin_.tau - (C_pin*dq + g_pin) :\n" << (data_pin_.tau - (C_pin*dq + g_pin)).transpose());
    ROS_INFO_STREAM("\ndiff oM4 :\n" << oM4_fra - oM4_pin);
    ROS_INFO_STREAM("\ndiff oM7 :\n" << oM7_fra - oM7_pin);
    ROS_INFO_STREAM("\ndiff coriolis vector :\n" << (cor_fra - C_pin*dq).transpose());
    ROS_INFO_STREAM("\ndiff mass matrix :\n" << M_fra - M_pin);
    ROS_INFO_STREAM("\nM_pin ONLY :\n" << M_pin);
    // ROS_INFO_STREAM("\nM_fra :\n" << M_fra);
    // ROS_INFO_STREAM("\nM_pin :\n" << M_pin);
    ROS_INFO_STREAM("\ndiff oM4 :\n" << oM4_fra - oM4_pin);
    ROS_INFO_STREAM("\ndiff lJ4_pin_f - lJ4_pin_j :\n" << lJ4_pin_f - lJ4_pin_j);
    ROS_INFO_STREAM("\ndiff lJ4_pin_f - lJ4_fra_j :\n" << lJ4_pin_f - lJ4_fra_j);
    ROS_INFO_STREAM("\ndiff oJ7_pin_f - oJ7_fra_j :\n" << oJ7_pin_f - oJ7_fra_j);
    ROS_INFO_STREAM("\ndiff loJ7_pin_f - oJ7_fra_j :\n" << loJ7_pin_f - oJ7_fra_j);
  }
}

}  // namespace panda_torque_mpc

PLUGINLIB_EXPORT_CLASS(panda_torque_mpc::ModelPinocchioVsFrankaController,
                       controller_interface::ControllerBase)
