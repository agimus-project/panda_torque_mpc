
// Adapted from panda_torque_mpc model_example_controller 
#pragma once

#include <memory>
#include <string>

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
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_hw/trigger_rate.h>

#include "panda_torque_mpc/common.h"


namespace panda_torque_mpc {

namespace pin = pinocchio;

class ModelPinocchioVsFrankaController
    : public controller_interface::MultiInterfaceController<franka_hw::FrankaModelInterface,
                                                            franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void update(const ros::Time&, const ros::Duration&) override;

 private:
  // Handles  
  std::unique_ptr<franka_hw::FrankaStateHandle> franka_state_handle_;
  std::unique_ptr<franka_hw::FrankaModelHandle> franka_model_handle_;

  franka_hw::TriggerRate rate_trigger_{1.0};

  // Pinocchio objects
  pin::Model model_pin_;
  pin::Data data_pin_;

};

} // namespace panda_torque_mpc
