#pragma once

#include <memory>
#include <string>
#include <vector>
#include <math.h>

#include <pinocchio/fwd.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/frames.hpp>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_box.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <geometry_msgs/PoseStamped.h>

#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/trigger_rate.h>

#include "panda_torque_mpc/JointValuesComparison.h"
#include "panda_torque_mpc/TaskPoseComparison.h"
#include "panda_torque_mpc/TaskTwistComparison.h"
#include "panda_torque_mpc/PoseTaskGoal.h"

#include "panda_torque_mpc/common.h"

#include "crocoddyl_reaching.h"



namespace panda_torque_mpc
{

    namespace pin = pinocchio;

    class CtrlMpcCroco : public controller_interface::MultiInterfaceController<franka_hw::FrankaModelInterface,
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
        Vector6d delta_nu_, period_nu_; // trajectory specification
        double alpha_dq_filter_;

        // Current update state
        pin::SE3 last_x_r_;
        Vector7d last_q_r_;
        Vector7d last_dq_r_;
        Vector7d last_tau_d_{};
        Vector7d dq_filtered_;

        // initial values
        ros::Time t_init_;
        Vector7d q_init_;
        pin::SE3 T_b_e0_;

        // Publishers
        franka_hw::TriggerRate rate_trigger_{1.0};
        realtime_tools::RealtimePublisher<TaskPoseComparison> task_pose_publisher_;
        realtime_tools::RealtimePublisher<TaskTwistComparison> task_twist_publisher_;
        realtime_tools::RealtimePublisher<JointValuesComparison> torques_publisher_;

        // Subscribers
        ros::Subscriber pose_subscriber_;
        bool use_external_pose_publisher_;
        pin::SE3 T_w_t0_; // initial value of broadcasted absolute pose
        bool pose_frames_not_aligned_;
        realtime_tools::RealtimeBox<pin::SE3> x_r_rtbox_;

        // Pinocchio objects
        pin::Model model_pin_;
        pin::Data data_pin_;

        // MPC formulation
        CrocoddylReaching croco_reaching_;
        CrocoddylConfig config_croco_;

        // other
        std::string ee_frame_pin_;
        pin::FrameIndex ee_frame_id_;

        /**
         * \brief Compute torque required to achieve end effector pose trajectory tracking.
         *
         * @param[in] q_m measured joint configuration
         * @param[in] dq_m measured joint velocity
         * @param[in] dq_filtered filtered joint velocity
         * @param[in] x_r target end effector pose
         */
        Vector7d compute_desired_torque(
            const Vector7d &q_m, const Vector7d &dq_m, const Vector7d &dq_filtered, const pin::SE3 &x_r, const CrocoddylConfig&);

        void pose_callback(const PoseTaskGoal& msg);

    };

} // namespace panda_torque_mpc
