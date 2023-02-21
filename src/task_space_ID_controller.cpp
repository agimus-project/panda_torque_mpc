#include "panda_torque_mpc/task_space_ID_controller.h"

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka/robot_state.h>

namespace panda_torque_mpc {

bool TaskSpaceIDController::init(hardware_interface::RobotHW* robot_hw,
                                          ros::NodeHandle& node_handle) {

  ///////////////////
  // Load parameters
  ///////////////////
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR("TaskSpaceIDController: Could not read parameter arm_id");
    return false;
  }

  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR("TaskSpaceIDController: Invalid or no joint_names parameters provided, aborting controller init!");
    return false;
  }

  if (!node_handle.getParam("Kp", Kp_)) {
    ROS_ERROR("TaskSpaceIDController: Could not read parameter Kp");
    return false;
  }

  if (!node_handle.getParam("Kd", Kd_)) {
    ROS_ERROR("TaskSpaceIDController: Could not read parameter Kd");
    return false;
  }

  if (!node_handle.getParam("w_posture", w_posture_)) {
    ROS_ERROR("TaskSpaceIDController: Could not read parameter w_posture");
    return false;
  }

  std::vector<double> delta_nu;
  if (!node_handle.getParam("delta_nu", delta_nu) || delta_nu.size() != 6) {
    ROS_ERROR("TaskSpaceIDController:  Invalid or no delta_nu parameters provided, aborting controller init!");
    return false;
  }
  delta_nu_ = Eigen::Map<Vector6d>(delta_nu.data());

  std::vector<double> period_nu;
  if (!node_handle.getParam("period_nu", period_nu) || period_nu.size() != 6) {
    ROS_ERROR("TaskSpaceIDController:  Invalid or no period_nu parameters provided, aborting controller init!");
    return false;
  }
  period_nu_ = Eigen::Map<Vector6d>(period_nu.data());


  double publish_rate(30.0);
  if (!node_handle.getParam("publish_rate", publish_rate)) {
    ROS_INFO_STREAM("TaskSpaceIDController: publish_rate not found. Defaulting to " << publish_rate);
  }
  rate_trigger_ = franka_hw::TriggerRate(publish_rate);

  int idc;
  if (!node_handle.getParam("control_variant", idc)  || !(idc >= 0 && idc < 3)) {
    ROS_ERROR_STREAM("TaskSpaceIDController: Invalid or no control_variant parameters provided, aborting controller init! control_variant: " << idc);
  }
  control_variant_ = static_cast<TaskSpaceIDController::TSIDVariant>(idc);

  if (!node_handle.getParam("use_pinocchio", use_pinocchio_)) {
    ROS_ERROR_STREAM("TaskSpaceIDController: Could not read parameter use_pinocchio");
  }

  if (!node_handle.getParam("alpha_dq_filter", alpha_dq_filter_)) {
    ROS_ERROR_STREAM("TaskSpaceIDController: Could not read parameter alpha_dq_filter");
  }

  // Load panda model with pinocchio
  std::string urdf_path;
  if (!node_handle.getParam("urdf_path", urdf_path)) {
    ROS_ERROR("TaskSpaceIDController: Could not read parameter urdf_path");
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

  // Define corresponding frame id for pinocchio and Franka (see model_pinocchio_vs_franka_controller)
  franka_frame_ = franka::Frame::kFlange;
  ee_frame_pin_ = "panda_link8";


  /////////////////////////////////////////////////
  /////////////////////////////////////////////////
  //                    TSID                     //
  /////////////////////////////////////////////////
  /////////////////////////////////////////////////

  tsid_reaching_ = TsidReaching(model_pin_, ee_frame_pin_, Kp_, Kd_, w_posture_);


  /////////////////////////////////////////////////
  /////////////////////////////////////////////////
  /////////////////////////////////////////////////
  /////////////////////////////////////////////////


  ///////////////////
  // Claim interfaces
  ///////////////////
  // Retrieve resource FrankaStateHandle
  auto* franka_state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (franka_state_interface == nullptr) {
    ROS_ERROR("TaskSpaceIDController: Could not get Franka state interface from hardware");
    return false;
  }
  try {
    franka_state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(franka_state_interface->getHandle(arm_id + "_robot"));
  } catch (const hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM("TaskSpaceIDController: Exception getting franka state handle: " << ex.what());
    return false;
  }

  // Retrieve resource FrankaModelHandle
  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM("TaskSpaceIDController: Error getting model interface from hardware");
    return false;
  }
  try {
    franka_model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM("TaskSpaceIDController: Exception getting model handle from interface: " << ex.what());
    return false;
  }

  // Retrieve resource FrankaModelHandle
  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM("TaskSpaceIDController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM("TaskSpaceIDController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  task_pose_publisher_.init(node_handle, "task_pose_comparison", 1);
  task_twist_publisher_.init(node_handle, "task_twist_comparison", 1);
  torques_publisher_.init(node_handle, "joint_torques_comparison", 1);

  dq_filtered_ = Vector7d::Zero();

  return true;
}

void TaskSpaceIDController::starting(const ros::Time& t0) {
  ROS_INFO_STREAM("TaskSpaceIDController::starting");
  t_init_ = t0;
  q_init_ = Eigen::Map<const Vector7d>(franka_state_handle_->getRobotState().q.data());
  pin::forwardKinematics(model_pin_, data_pin_, q_init_);
  pin::updateFramePlacements(model_pin_, data_pin_);
  x_init_ = data_pin_.oMf[model_pin_.getFrameId(ee_frame_pin_)];

  tsid_reaching_.setPostureRef(q_init_);

  ROS_INFO_STREAM("TaskSpaceIDController::starting x_init_: \n" << x_init_);
}


void TaskSpaceIDController::update(const ros::Time& t, const ros::Duration& period) {

  // Time since start of the controller
  double Dt = (t - t_init_).toSec();

  // compute desired configuration and configuration velocity
  pin::SE3 x_r; 
  pin::Motion dx_r, ddx_r;
  compute_sinusoid_pose_reference(delta_nu_, period_nu_, x_init_, Dt, x_r, dx_r, ddx_r);

  // Retrieve current measured robot state
  franka::RobotState robot_state = franka_state_handle_->getRobotState();  // return a const& of RobotState object -> not going to be modified
  Eigen::Map<Vector7d> q_m(robot_state.q.data());
  Eigen::Map<Vector7d> dq_m(robot_state.dq.data());
  Eigen::Map<Vector7d> tau_m(robot_state.tau_J.data());  // measured torques -> naturally contains gravity torque
  // End effector computed state
  // FK and differential FK
  pin::forwardKinematics(model_pin_, data_pin_, q_m, dq_m);
  pin::updateFramePlacements(model_pin_, data_pin_);
  auto fid = model_pin_.getFrameId(ee_frame_pin_);
  pin::SE3 T_o_e_m = data_pin_.oMf[fid];
  pin::Motion nu_o_e_m = pin::getFrameVelocity(model_pin_, data_pin_, fid, pin::LOCAL_WORLD_ALIGNED);


  // filter the joint velocity measurements
  dq_filtered_ = (1 - alpha_dq_filter_) * dq_filtered_ + alpha_dq_filter_ * dq_m;

  // Compute desired torque
  Vector7d tau_d = compute_desired_torque(q_m, dq_m, dq_filtered_, x_r, dx_r, ddx_r, control_variant_, use_pinocchio_);

  // Maximum torque difference with a sampling rate of 1 kHz. The maximum torque rate is
  // 1000 * (1 / sampling_time).
  // Vector7d tau_d_saturated = saturateTorqueRate(tau_d, Eigen::Map<Vector7d>(robot_state.tau_J_d.data()));
  Vector7d tau_d_saturated = tau_d;

  // Send Torque Command
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d_saturated[i]);
  }

  ///////////////////
  // Publish logs
  if (rate_trigger_() && torques_publisher_.trylock()) {

    // Refactor to: publish_logs(publishers, last_vals, measured_vals) ?

    /**
     * Measured torque in simulation returns -tau while running with real robot returns tau --___--
     */
    tau_m = -tau_m;  // SIMULATION
    // tau_m = tau_m;  // REAL

    // torque
    Vector7d tau_error = last_tau_d_ - tau_m;
    // EE pose
    Eigen::Vector3d p_o_e_err = T_o_e_m.translation() - last_x_r_.translation();
    Eigen::Quaterniond quat_r(last_x_r_.rotation());
    Eigen::Quaterniond quat_m(T_o_e_m.rotation());
    Eigen::Quaterniond quat_err = quat_r.inverse() * quat_m;
    // EE twist
    Eigen::Vector3d v_o_e_err = nu_o_e_m.linear() - dx_r.linear();
    Eigen::Vector3d omg_o_e_err = nu_o_e_m.angular() - dx_r.angular();

    // EE Twists linear part
    task_twist_publisher_.msg_.commanded.linear.x = dx_r.linear()[0];
    task_twist_publisher_.msg_.commanded.linear.y = dx_r.linear()[1];
    task_twist_publisher_.msg_.commanded.linear.z = dx_r.linear()[2];
    task_twist_publisher_.msg_.measured.linear.x = nu_o_e_m.linear()[0];
    task_twist_publisher_.msg_.measured.linear.y = nu_o_e_m.linear()[1];
    task_twist_publisher_.msg_.measured.linear.z = nu_o_e_m.linear()[2];
    task_twist_publisher_.msg_.error.linear.x = v_o_e_err[0];
    task_twist_publisher_.msg_.error.linear.y = v_o_e_err[1];
    task_twist_publisher_.msg_.error.linear.z = v_o_e_err[2];

    // EE Twists angular part
    task_twist_publisher_.msg_.commanded.angular.x = dx_r.angular()[0];
    task_twist_publisher_.msg_.commanded.angular.y = dx_r.angular()[1];
    task_twist_publisher_.msg_.commanded.angular.z = dx_r.angular()[2];
    task_twist_publisher_.msg_.measured.angular.x = nu_o_e_m.angular()[0];
    task_twist_publisher_.msg_.measured.angular.y = nu_o_e_m.angular()[1];
    task_twist_publisher_.msg_.measured.angular.z = nu_o_e_m.angular()[2];
    task_twist_publisher_.msg_.error.angular.x = omg_o_e_err[0];
    task_twist_publisher_.msg_.error.angular.y = omg_o_e_err[1];
    task_twist_publisher_.msg_.error.angular.z = omg_o_e_err[2];

    // End effector position
    task_pose_publisher_.msg_.commanded.position.x = x_r.translation()[0];
    task_pose_publisher_.msg_.commanded.position.y = x_r.translation()[1];
    task_pose_publisher_.msg_.commanded.position.z = x_r.translation()[2];
    task_pose_publisher_.msg_.measured.position.x = T_o_e_m.translation()[0];
    task_pose_publisher_.msg_.measured.position.y = T_o_e_m.translation()[1];
    task_pose_publisher_.msg_.measured.position.z = T_o_e_m.translation()[2];
    task_pose_publisher_.msg_.error.position.x = p_o_e_err[0];
    task_pose_publisher_.msg_.error.position.y = p_o_e_err[1];
    task_pose_publisher_.msg_.error.position.z = p_o_e_err[2];

    // Quaternion
    task_pose_publisher_.msg_.commanded.orientation.x = quat_r.x();
    task_pose_publisher_.msg_.commanded.orientation.y = quat_r.y();
    task_pose_publisher_.msg_.commanded.orientation.z = quat_r.z();
    task_pose_publisher_.msg_.commanded.orientation.w = quat_r.w();
    task_pose_publisher_.msg_.measured.orientation.x = quat_m.x();
    task_pose_publisher_.msg_.measured.orientation.y = quat_m.y();
    task_pose_publisher_.msg_.measured.orientation.z = quat_m.z();
    task_pose_publisher_.msg_.measured.orientation.w = quat_m.w();
    task_pose_publisher_.msg_.error.orientation.x = quat_err.x();
    task_pose_publisher_.msg_.error.orientation.y = quat_err.y();
    task_pose_publisher_.msg_.error.orientation.z = quat_err.z();
    task_pose_publisher_.msg_.error.orientation.w = quat_err.w();

    // Size 7 vectors
    for (size_t i = 0; i < 7; ++i) {
      torques_publisher_.msg_.commanded[i] = last_tau_d_[i];
      torques_publisher_.msg_.measured[i] = tau_m[i];
      torques_publisher_.msg_.error[i] = tau_error[i];
    }

    task_pose_publisher_.unlockAndPublish();
    task_twist_publisher_.unlockAndPublish();
    torques_publisher_.unlockAndPublish();
  }

  // Store previous desired/reference values
  last_x_r_ = x_r; 
  last_dx_r_ = dx_r; 
  last_tau_d_ = tau_d_saturated + Eigen::Map<Vector7d>(franka_model_handle_->getGravity().data());
}


Vector7d TaskSpaceIDController::compute_desired_torque(
      const Vector7d& q_m, const Vector7d& dq_m, const Vector7d& dq_filtered, 
      const pin::SE3& x_r, const pin::Motion& dx_r, const pin::Motion& ddx_r, 
      TSIDVariant control_variant, bool use_pinocchio)
{

  /** 
   * 
   * Lagrangian dynamics:
   * M(q)*ddq + C(q,dq)*dq + g(q) = tau + J^T*f
   * 
   * Here, we assume no contact so f=0.
   * 
   * 
   * TSID -> feedback at the task acceleration level (end effector reference tracking) 
   * ddx_d = ddx_r − Kd ( dx_m − dx_r) − Kp(x_m − x_r)
   * where dx_m and x_m come from Forward kinematics
   * 
   * if x is R^3 (EE position control) -> x_m - x_r: simple euclidean diff
   * BUT, if x is SE3 (EE pose control) -> x_m - x_r = log(x_r^-1 * x) 
   * 
   * ddq_d = J^# (ddx_d - dJ*dq_m)
   * Or solve
   * J*ddq_d = ddx_d - dJ*dq_m
   * tau_d = M ddq_d + h(q_m, dq_m) = rnea(q_m, dq_m, ddq_d) 
   */


  /**
   * For differential Forward Kinematics and jacobians, use LOCAL_WORL_ALIGNED 
   * for all computations since corresponds to the "classical" euclidean velocity (easier to think about)
   * 
  */ 
  // FK and differential FK
  pin::forwardKinematics(model_pin_, data_pin_, q_m, dq_m);
  pin::updateFramePlacements(model_pin_, data_pin_);
  auto fid = model_pin_.getFrameId(ee_frame_pin_);
  pin::SE3 T_o_e_m = data_pin_.oMf[fid];
  pin::Motion nu_o_e_m = pin::getFrameVelocity(model_pin_, data_pin_, fid, pin::LOCAL_WORLD_ALIGNED);

  // end effector jacobian and time derivative
  Eigen::Matrix<double, 6, 7> J_pin, dJ_pin;
  J_pin.setZero(); 
  pin::computeFrameJacobian(model_pin_, data_pin_, q_m, fid, pin::LOCAL_WORLD_ALIGNED, J_pin); 
  dJ_pin.setZero();
  pin::computeJointJacobiansTimeVariation(model_pin_, data_pin_, q_m, dq_m);
  pin::getFrameJacobianTimeVariation(model_pin_, data_pin_, fid, pin::LOCAL_WORLD_ALIGNED, dJ_pin);

  Vector7d ddq_d;  // desired joint acceleration
  switch(control_variant) {
    case TSIDVariant::PosiPosture:
      {
      ROS_INFO_STREAM("TSIDVariant::PosiPosture, pinocchio: " << use_pinocchio_);
      // ////////////
      // // POSITION ONLY
      // // UNSTABLE cause UNDERTERMINED!! (3 < 7 Dof constrained)
      // Eigen::Vector3d e = T_o_e_m.translation() - x_r.translation();
      // Eigen::Vector3d de = nu_o_e_m.linear() - dx_r.linear();
      // Eigen::Vector3d ddx_d = ddx_r.linear() - Kd_ * de - Kp_ * e;

      // Eigen::Matrix<double,3,7> A = J_pin.block<3,7>(0,0);
      // Eigen::Matrix<double,3,1> b = ddx_d - dJ_pin.block<3,7>(0,0) * dq_m;
      // ddq_d = A.colPivHouseholderQr().solve(b);
      // ///////////////////////

      ////////////
      // EE POSITION + POSTURE tasks
      // Position task
      Eigen::Vector3d ex = T_o_e_m.translation() - x_r.translation();
      Eigen::Vector3d dex = nu_o_e_m.linear() - dx_r.linear();
      Eigen::Vector3d ddx_d = ddx_r.linear() - Kd_ * dex - Kp_ * ex;

      // Posture task : q --> q_init
      // Let's keep the same dynamics but alpha will handle the weighting between the 2 tasks
      // ddq_r = 0 = dq_r here 
      Vector7d eq = q_m - q_init_;
      Vector7d deq = dq_m;
      Vector7d ddq_reg_d = - Kp_ * eq - Kd_ * deq;

      // Create and solve least square problem to get desired joint acceleration
      Eigen::Matrix<double,10,7> A; 
      A.block<3,7>(0,0) = J_pin.block<3,7>(0,0);
      A.block<7,7>(3,0) = pow(w_posture_,2) * Eigen::Matrix<double,7,7>::Identity();
      Eigen::Matrix<double,10,1> b; 
      b.segment<3>(0) = ddx_d - dJ_pin.block<3,7>(0,0) * dq_m;
      b.segment<7>(3) = pow(w_posture_,2) * ddq_reg_d;
      ddq_d = A.colPivHouseholderQr().solve(b);

      break;
      }
    case TSIDVariant::PosePosture: 
      {
      ROS_INFO_STREAM("TSIDVariant::PosePosture, pinocchio: " << use_pinocchio_);

      // EE SE3 POSE + POSTURE tasks  --> NOPE, likely problem with the jacobian computation
      // End effector pose task
      // pin::SE3 e = x_r.inverse() * T_o_e_m;
      pin::SE3 e = T_o_e_m.inverse() * x_r;
      pin::Motion de = nu_o_e_m - dx_r;
      Vector6d ddx_d = ddx_r - Kd_*de - Kp_*pin::log6(e);
      Eigen::Matrix<double, 6, 6> Jlog; pin::Jlog6(e, Jlog);
        
      // Posture task : q --> q_init
      // Let's keep the same dynamics but alpha will handle the weighting between the 2 tasks
      // ddq_r = 0 = dq_r here 
      Vector7d eq = q_m - q_init_;
      Vector7d deq = dq_m;
      Vector7d ddq_reg_d = - Kp_ * eq - Kd_ * deq;

      // SE3 ONLY -> UNDERDETERMINED
      // Eigen::Matrix<double, 6, 7> A = Jlog * J_pin;
      // Eigen::Matrix<double, 6, 7> b = ddx_d - dJ_pin * dq_m;
      // ddq_d = A.colPivHouseholderQr().solve(b);

      // Create and solve least square problem to get desired joint acceleration
      Eigen::Matrix<double,13,7> A; 
      A.block<6,7>(0,0) = Jlog * J_pin;
      A.block<7,7>(6,0) = pow(w_posture_,2) * Eigen::Matrix<double,7,7>::Identity();
      Eigen::Matrix<double,13,1> b; 
      b.segment<6>(0) = ddx_d - dJ_pin * dq_m;
      b.segment<7>(6) = pow(w_posture_,2) * ddq_reg_d;
      ddq_d = A.colPivHouseholderQr().solve(b);

      break;
      }
    case TSIDVariant::TSID:
      ROS_INFO_STREAM("TSIDVariant::TSID, pinocchio: " << use_pinocchio_);

      tsid_reaching_.setEERef(x_r, dx_r, ddx_r);
      tsid_reaching_.solve(q_m, dq_m);
      ddq_d = tsid_reaching_.getAccelerations();
      // tau_d = tsid_reaching.getTorques();
      
      break;

  }





  Vector7d tau_d;
  if (use_pinocchio_){
    pin::rnea(model_pin_, data_pin_, q_m, dq_m, ddq_d);
    pin::computeGeneralizedGravity(model_pin_, data_pin_, q_m);  // data.g == generalized gravity

    tau_d = data_pin_.tau - data_pin_.g;
  }

  else {
    // libfranka dynamics
    std::array<double, 7> coriolis_fra_arr = franka_model_handle_->getCoriolis();
    std::array<double, 7> gravity_fra_arr = franka_model_handle_->getGravity();
    std::array<double, 49> M_fra_arr = franka_model_handle_->getMass();
    // Eigen and Franka use Column-Major storage order (see model_pinocchio_vs_frank_controller)
    Eigen::Map<Vector7d> coriolis_fra(coriolis_fra_arr.data());  // C(q,dq)*dq
    Eigen::Map<Vector7d> gravity_fra(gravity_fra_arr.data());    // g(q)
    Eigen::Map<Matrix7d> M_fra(M_fra_arr.data());                // M(q)

    tau_d = M_fra * ddq_d + coriolis_fra;
  }  


  return tau_d;
}

void TaskSpaceIDController::compute_sinusoid_pose_reference(const Vector6d& delta_nu, const Vector6d& period_nu, const pin::SE3& pose_0, double t,
                                                            pin::SE3& x_r, pin::Motion& dx_r, pin::Motion& ddx_r) 
{ 
  // Ai and Ci obtained for each joint using constraints: 
  // T(t=0.0) = pose_0
  // T(t=period/2) = pose_0 * Exp(delta_nu)
  
  Vector6d w = (2*M_PI/period_nu.array()).matrix();
  Vector6d a = - delta_nu;
  Vector6d c = delta_nu;

  Vector6d nu =                                 (a.array()*cos(w.array()*t)).matrix() + c;
  dx_r        = pin::Motion(         (-w.array()*a.array()*sin(w.array()*t)).matrix());
  ddx_r       = pin::Motion((-w.array().square()*a.array()*cos(w.array()*t)).matrix());  // non null initial acceleration!! needs to be dampened (e.g. torque staturation)

  // ROS_INFO_STREAM("TaskSpaceIDController::compute_sinusoid_pose_reference pose_0: \n" << pose_0);
  // ROS_INFO_STREAM("TaskSpaceIDController::compute_sinusoid_pose_reference nu: \n" << nu.transpose());
  // ROS_INFO_STREAM("TaskSpaceIDController::compute_sinusoid_pose_reference pin::exp6(nu): \n" << pin::exp6(nu));

  x_r = pose_0 * pin::exp6(nu);
  // ROS_INFO_STREAM("TaskSpaceIDController::compute_sinusoid_pose_reference x_r: \n" << x_r);
}

Vector7d TaskSpaceIDController::saturateTorqueRate(
    const Vector7d& tau_d,
    const Vector7d& tau_J_d) {  // NOLINT (readability-identifier-naming)
  Vector7d tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d[i] - tau_J_d[i];
    tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, kDeltaTauMax_), -kDeltaTauMax_);
  }
  return tau_d_saturated;
}




void TaskSpaceIDController::stopping(const ros::Time& t0) {
  ROS_INFO_STREAM("TaskSpaceIDController::stopping");
}

}  // namespace panda_torque_mpc

PLUGINLIB_EXPORT_CLASS(panda_torque_mpc::TaskSpaceIDController,
                       controller_interface::ControllerBase)
