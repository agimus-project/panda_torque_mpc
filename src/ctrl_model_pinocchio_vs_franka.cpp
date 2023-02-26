#include "panda_torque_mpc/ctrl_model_pinocchio_vs_franka.h"

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

namespace
{
    template <class T, size_t N>
    std::ostream &operator<<(std::ostream &ostream, const std::array<T, N> &array)
    {
        ostream << "[";
        std::copy(array.cbegin(), array.cend() - 1, std::ostream_iterator<T>(ostream, ","));
        std::copy(array.cend() - 1, array.cend(), std::ostream_iterator<T>(ostream));
        ostream << "]";
        return ostream;
    }
} // anonymous namespace

namespace panda_torque_mpc
{

    bool CtrlModelPinocchioVsFranka::init(hardware_interface::RobotHW *robot_hw,
                                                ros::NodeHandle &node_handle)
    {

        ///////////////////
        // Load parameters
        ///////////////////
        std::string arm_id;
        if (!node_handle.getParam("arm_id", arm_id))
        {
            ROS_ERROR("CtrlModelPinocchioVsFranka: Could not read parameter arm_id");
            return false;
        }

        // Load Pinocchio urdf
        std::string urdf_path;
        if (!node_handle.getParam("urdf_path", urdf_path))
        {
            ROS_ERROR("CtrlModelPinocchioVsFranka: Could not read parameter urdf_path");
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
        auto *franka_state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
        if (franka_state_interface == nullptr)
        {
            ROS_ERROR("CtrlModelPinocchioVsFranka: Could not get Franka state interface from hardware");
            return false;
        }
        try
        {
            franka_state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(franka_state_interface->getHandle(arm_id + "_robot"));
        }
        catch (const hardware_interface::HardwareInterfaceException &ex)
        {
            ROS_ERROR_STREAM("CtrlModelPinocchioVsFranka: Exception getting franka state handle: " << ex.what());
            return false;
        }

        // Retrieve resource FrankaModelHandle
        auto *model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
        if (model_interface == nullptr)
        {
            ROS_ERROR_STREAM("CtrlModelPinocchioVsFranka: Error getting model interface from hardware");
            return false;
        }
        try
        {
            franka_model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(model_interface->getHandle(arm_id + "_model"));
        }
        catch (hardware_interface::HardwareInterfaceException &ex)
        {
            ROS_ERROR_STREAM("CtrlModelPinocchioVsFranka: Exception getting model handle from interface: " << ex.what());
            return false;
        }

        return true;
    }

    void CtrlModelPinocchioVsFranka::update(const ros::Time & /*time*/, const ros::Duration & /*period*/)
    {
        if (rate_trigger_())
        {
            // FRANKA MODEL
            // -Frank Rigid Body Dynamics computations are done on "Control" computer, libfranka only communicates
            // with it to retrieve data.
            // - What is the definition of franka::Frame::kEndEffector?
            std::array<double, 49> mass = franka_model_handle_->getMass();
            std::array<double, 7> gravity = franka_model_handle_->getGravity();
            std::array<double, 7> coriolis = franka_model_handle_->getCoriolis();
            std::array<double, 16> pose_j4 = franka_model_handle_->getPose(franka::Frame::kJoint4);
            std::array<double, 16> pose_j7 = franka_model_handle_->getPose(franka::Frame::kJoint7);
            std::array<double, 16> pose_fl = franka_model_handle_->getPose(franka::Frame::kFlange);
            std::array<double, 16> pose_ee = franka_model_handle_->getPose(franka::Frame::kEndEffector);
            std::array<double, 42> joint4_body_jacobian = franka_model_handle_->getBodyJacobian(franka::Frame::kJoint4);
            std::array<double, 42> joint7_zero_jacobian = franka_model_handle_->getZeroJacobian(franka::Frame::kJoint7);

            // Retrieve current robot state from state handle
            franka::RobotState robot_state = franka_state_handle_->getRobotState();
            Eigen::Map<Vector7d> q(robot_state.q.data());
            Eigen::Map<Vector7d> dq(robot_state.dq.data());
            Vector7d ddq = Vector7d::Zero();

            // pinocchio computations (stored in data_pin_)
            // M(q)*ddq + C(q,dq)*dq + g(q) = tau + J^T*fext
            pin::forwardKinematics(model_pin_, data_pin_, q, dq);     // joint frame placements
            pin::updateFramePlacements(model_pin_, data_pin_);        // link frame placements
            pin::computeCoriolisMatrix(model_pin_, data_pin_, q, dq); // C(q, dq)
            pin::computeGeneralizedGravity(model_pin_, data_pin_, q); // g(q)
            pin::crba(model_pin_, data_pin_, q);                      // M(q)
            pin::rnea(model_pin_, data_pin_, q, dq, ddq);             // data.tau

            // Pinocchio frame ids: joints are a subset of all frames, their ids correspond to the kinematic chain ordee.
            // Pinocchio joint ids: id in the collection of non-fixed joints. The list starts always with universe, so first real joint has id 1.
            auto jid_j4 = model_pin_.getJointId("panda_joint4");
            auto jid_j7 = model_pin_.getJointId("panda_joint7");
            auto fid_j4 = model_pin_.getFrameId("panda_joint4");
            auto fid_j7 = model_pin_.getFrameId("panda_joint7");
            auto fid_f7 = model_pin_.getFrameId("panda_link7");
            auto fid_f8 = model_pin_.getFrameId("panda_link8");

            ////////////////////////////////
            // Store everything in Eigen::Matrix objects for easy comparison
            // Eigen and Franka use Column-Major storage order

            // Pose as homogeneous transformation matrices
            Eigen::Matrix<double, 4, 4> oMj4_pin = data_pin_.oMi.at(jid_j4).toHomogeneousMatrix();
            Eigen::Matrix<double, 4, 4> oMj7_pin = data_pin_.oMi.at(jid_j7).toHomogeneousMatrix();
            Eigen::Matrix<double, 4, 4> oMf7_pin = data_pin_.oMf.at(fid_f7).toHomogeneousMatrix();
            Eigen::Matrix<double, 4, 4> oMf8_pin = data_pin_.oMf.at(fid_f8).toHomogeneousMatrix();
            Eigen::Map<Eigen::Matrix<double, 4, 4>> oM4_fra(pose_j4.data());
            Eigen::Map<Eigen::Matrix<double, 4, 4>> oM7_fra(pose_j7.data());
            Eigen::Map<Eigen::Matrix<double, 4, 4>> oMfl_fra(pose_fl.data());
            Eigen::Map<Eigen::Matrix<double, 4, 4>> oMee_fra(pose_ee.data());
            // Frame jacobians Ji st. vi = Ji * q
            // !! call setZero since pinocchio does not internally. Unitialized values appear for frames in the middle of the kinematic chain
            Eigen::Matrix<double, 6, 7> lJ4_pin_f;
            lJ4_pin_f.setZero();
            pin::computeFrameJacobian(model_pin_, data_pin_, q, fid_j4, lJ4_pin_f);
            // computeJointJacobian has no diff if corresponding joints AND computeFrameJacobian called with pin::LOCAL (default)
            Eigen::Matrix<double, 6, 7> lJ4_pin_j;
            lJ4_pin_j.setZero();
            pin::computeJointJacobian(model_pin_, data_pin_, q, jid_j4, lJ4_pin_j);
            Eigen::Matrix<double, 6, 7> oJ7_pin_f;
            oJ7_pin_f.setZero();
            pin::computeFrameJacobian(model_pin_, data_pin_, q, fid_j7, pin::WORLD, oJ7_pin_f);
            Eigen::Matrix<double, 6, 7> loJ7_pin_f;
            loJ7_pin_f.setZero();
            pin::computeFrameJacobian(model_pin_, data_pin_, q, fid_j7, pin::LOCAL_WORLD_ALIGNED, loJ7_pin_f);
            Eigen::Map<Eigen::Matrix<double, 6, 7>> lJ4_fra_j(joint4_body_jacobian.data());
            Eigen::Map<Eigen::Matrix<double, 6, 7>> oJ7_fra_j(joint7_zero_jacobian.data());
            // Lagrangian dynamics equation elements
            Vector7d g_pin = data_pin_.g;
            Matrix7d C_pin = data_pin_.C;
            Matrix7d M_pin = data_pin_.M;
            // crba only fills upper triangular part of the mass matrix, let's fill the lower triangular part
            M_pin.triangularView<Eigen::StrictlyLower>() = M_pin.transpose().triangularView<Eigen::StrictlyLower>();
            Eigen::Map<Vector7d> g_fra(gravity.data());
            Eigen::Map<Vector7d> cor_fra(coriolis.data());
            Eigen::Map<Matrix7d> M_fra(mass.data());

            ROS_INFO("\n\n\n--------------------------------------------------");
            // std::cout << "\ndiff coriolis: \n" << coriolis);
            std::cout << "\ngravity_fra :\n"
                      << (g_fra).transpose();
            std::cout << "\ngravity_pin :\n"
                      << (g_pin).transpose();
            std::cout << "\ndiff gravity :\n"
                      << (g_fra - g_pin).transpose(); // NOT SAME
            // std::cout << "\ngravity fra:\n" << g_fra.transpose();
            // std::cout << "\ngravity pin:\n" << g_pin.transpose();
            std::cout << "\ndiff data_pin_.tau - (C_pin*dq + g_pin) :\n"
                      << (data_pin_.tau - (C_pin * dq + g_pin)).transpose(); // SAME

            std::cout << "\ndiff coriolis vector :\n"
                      << (cor_fra - C_pin * dq).transpose(); // NOT SAME
            std::cout << "\ndiff mass matrix :\n"
                      << M_fra - M_pin; // NOT SAME
            // std::cout << "\nM_pin :\n" << M_pin);
            // std::cout << "\nM_fra :\n" << M_fra);
            std::cout << "\ndiff oM4_fra - oMj4_pin :\n"
                      << oM4_fra - oMj4_pin; // SAME
            std::cout << "\ndiff oM7_fra - oMj7_pin :\n"
                      << oM7_fra - oMj7_pin; // SAME
            std::cout << "\ndiff oMfl_fra - oMf8_pin :\n"
                      << oMfl_fra - oMf8_pin; // SAME
            std::cout << "\ndiff lJ4_pin_f - lJ4_pin_j :\n"
                      << lJ4_pin_f - lJ4_pin_j; // SAME
            std::cout << "\ndiff lJ4_pin_f - lJ4_fra_j :\n"
                      << lJ4_pin_f - lJ4_fra_j; // SAME
            // std::cout << "\ndiff oJ7_pin_f - oJ7_fra_j :\n" << oJ7_pin_f - oJ7_fra_j;    // NOT SAME: Franka "zero" Jac = LOCAL_WORLD_ALIGNED, not WORLD
            std::cout << "\ndiff loJ7_pin_f - oJ7_fra_j :\n"
                      << loJ7_pin_f - oJ7_fra_j; // SAME
        }
    }

} // namespace panda_torque_mpc

PLUGINLIB_EXPORT_CLASS(panda_torque_mpc::CtrlModelPinocchioVsFranka,
                       controller_interface::ControllerBase)
