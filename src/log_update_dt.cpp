#include "panda_torque_mpc/log_update_dt.h"

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

#include <fstream>

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

bool LogUpdateDt::init(hardware_interface::RobotHW* robot_hw,
                       ros::NodeHandle& node_handle) {

  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("ModelPinocchioVsFrankaController: Could not read parameter arm_id");
    return false;
  }

  ///////////////////
  // Claim interfaces
  ///////////////////
  // Retrieve resource FrankaStateHandle
  auto* franka_state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (franka_state_interface == nullptr) {
    ROS_ERROR("LogUpdateDt: Could not get Franka state interface from hardware");
    return false;
  }
  try {
    franka_state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(franka_state_interface->getHandle(arm_id + "_robot"));
  } catch (const hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM("LogUpdateDt: Exception getting franka state handle: " << ex.what());
    return false;
  }

  return true;
}

void LogUpdateDt::starting(const ros::Time& t0) {
  ROS_INFO_STREAM("LogUpdateDt::starting");
  t_init_ = t0;
  time_vec_.reserve(120*1000);
  dur_vec_.reserve(120*1000);
}



void LogUpdateDt::update(const ros::Time& t, const ros::Duration& dur) {

  time_vec_.push_back((t - t_init_).toSec());
  dur_vec_.push_back(dur.toSec());
}

void LogUpdateDt::stopping(const ros::Time& t0) {
  ROS_INFO_STREAM("LogUpdateDt::stopping");

  ///////////////////////
  // STORE DT
  std::ofstream myfile;
  myfile.open("test_panfa_time_vec.csv");
  myfile << "t" << "," << "dur" << "\n";
  for (unsigned int i=0;  i < time_vec_.size(); i++)
  {
    myfile << time_vec_[i] << "," << dur_vec_[i] << "\n";
  }
  myfile << std::setprecision(20);
  myfile.close();
  ROS_INFO_STREAM("Saved test_panfa_time_vec.csv");
  ///////////////////////
}

}  // namespace panda_torque_mpc

PLUGINLIB_EXPORT_CLASS(panda_torque_mpc::LogUpdateDt,
                       controller_interface::ControllerBase)
