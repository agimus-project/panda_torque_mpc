
// Adapted from panda_torque_mpc model_example_controller
#pragma once

#include <memory>
#include <string>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_hw/trigger_rate.h>

namespace panda_torque_mpc
{

    class CtrlLogUpdateDt
        : public controller_interface::MultiInterfaceController<franka_hw::FrankaStateInterface>
    {
    public:
        bool init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle) override;
        void starting(const ros::Time &) override;
        void update(const ros::Time &, const ros::Duration &) override;
        void stopping(const ros::Time &) override;

    private:
        // Handles
        std::unique_ptr<franka_hw::FrankaStateHandle> franka_state_handle_;

        // logging data
        std::vector<double> time_vec_;
        std::vector<double> dur_vec_;
        ros::Time t_init_;
    };

} // namespace panda_torque_mpc
