#include <panda_torque_mpc/task_space_ID_controller.h>

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
  pin_frame_ = "panda_link8";
  

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

  configurations_publisher_.init(node_handle, "joint_configurations_comparison", 1);
  velocities_publisher_.init(node_handle, "joint_velocities_comparison", 1);
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
  x_init_ = data_pin_.oMf[model_pin_.getFrameId(pin_frame_)];
  ROS_INFO_STREAM("TaskSpaceIDController::starting x_init_: \n" << x_init_);
}


void TaskSpaceIDController::update(const ros::Time& t, const ros::Duration& period) {

  // Time since start of the controller
  double Dt = (t - t_init_).toSec();

  // compute desired configuration and configuration velocity
  pin::SE3 x_r; 
  pin::Motion dx_r, ddx_r;
  compute_sinusoid_pose_reference(delta_nu_, period_nu_, x_init_, Dt, x_r, dx_r, ddx_r);

  ROS_INFO_STREAM("TaskSpaceIDController::update x_r: \n" << x_r);

  // Retrieve current measured robot state
  franka::RobotState robot_state = franka_state_handle_->getRobotState();  // return a const& of RobotState object -> not going to be modified
  Eigen::Map<Vector7d> q_m(robot_state.q.data());
  Eigen::Map<Vector7d> dq_m(robot_state.dq.data());
  Eigen::Map<Vector7d> tau_m(robot_state.tau_J.data());  // measured torques -> naturally contains gravity torque

  // filter the joint velocity measurements
  dq_filtered_ = (1 - alpha_dq_filter_) * dq_filtered_ + alpha_dq_filter_ * dq_m;

  // Compute desired torque
  Vector7d tau_d = compute_desired_torque(q_m, dq_m, dq_filtered_, x_r, dx_r, ddx_r, use_pinocchio_);

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

    // Vector7d q_error = last_q_r_ - q_m;
    // Vector7d dq_error = last_dq_r_ - dq_m;
    Vector7d tau_error = last_tau_d_ - tau_m;

    // double q_error_rms = std::sqrt(q_error.array().square().sum()) / 7.0;
    // double dq_error_rms = std::sqrt(dq_error.array().square().sum()) / 7.0;
    double tau_error_rms = std::sqrt(tau_error.array().square().sum()) / 7.0;
    // configurations_publisher_.msg_.root_mean_square_error = q_error_rms;
    // velocities_publisher_.msg_.root_mean_square_error = dq_error_rms;
    torques_publisher_.msg_.root_mean_square_error = tau_error_rms;
    
    for (size_t i = 0; i < 7; ++i) {
      // // Joint config
      // configurations_publisher_.msg_.commanded[i] = last_q_r_[i];
      // configurations_publisher_.msg_.measured[i] = q_m[i];
      // configurations_publisher_.msg_.error[i] = q_error[i];

      // // Joint velocities
      // velocities_publisher_.msg_.commanded[i] = last_dq_r_[i];
      // velocities_publisher_.msg_.measured[i] = dq_m[i];
      // velocities_publisher_.msg_.error[i] = dq_error[i];

      // Joint torque
      torques_publisher_.msg_.commanded[i] = last_tau_d_[i];
      torques_publisher_.msg_.measured[i] = tau_m[i];
      torques_publisher_.msg_.error[i] = tau_error[i];
    }

    // configurations_publisher_.unlockAndPublish();
    // velocities_publisher_.unlockAndPublish();
    torques_publisher_.unlockAndPublish();
  }

  // Store previous desired/reference values
  // last_q_r_ = q_r; 
  // last_dq_r_ = dq_r; 
  last_tau_d_ = tau_d_saturated + Eigen::Map<Vector7d>(franka_model_handle_->getGravity().data());
}


Vector7d TaskSpaceIDController::compute_desired_torque(
      const Vector7d& q_m, const Vector7d& dq_m, const Vector7d& dq_filtered, 
      const pin::SE3& x_r, const pin::Motion& dx_r, const pin::Motion& ddx_r, 
      bool use_pinocchio)
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
  auto fid = model_pin_.getFrameId(pin_frame_);
  pin::SE3 T_o_e_m = data_pin_.oMf[fid];
  pin::Motion nu_o_e_m = pin::getFrameVelocity(model_pin_, data_pin_, fid, pin::LOCAL_WORLD_ALIGNED);

  // end effector jacobian and time derivative
  Eigen::Matrix<double, 6, 7> J_pin, dJ_pin;
  J_pin.setZero(); 
  pin::computeFrameJacobian(model_pin_, data_pin_, q_m, fid, pin::LOCAL_WORLD_ALIGNED, J_pin); 
  dJ_pin.setZero();
  pin::computeJointJacobiansTimeVariation(model_pin_, data_pin_, q_m, dq_m);
  pin::getFrameJacobianTimeVariation(model_pin_, data_pin_, fid, pin::LOCAL_WORLD_ALIGNED, dJ_pin);


  // Create and solve least square problem to get desired joint acceleration
  
  //////////////
  // POSITION ONLY
  Eigen::Vector3d e = T_o_e_m.translation() - x_r.translation();
  Eigen::Vector3d de = nu_o_e_m.linear() - dx_r.linear();
  Eigen::Vector3d ddx_d = ddx_r.linear() - Kd_ * de - Kp_ * e;
  
  Eigen::MatrixXd A = J_pin.block<3,7>(0,0);
  Eigen::VectorXd b = ddx_d - dJ_pin.block<3,7>(0,0) * dq_m;
  Vector7d ddq_d = A.colPivHouseholderQr().solve(b);
  ///////////////////////

  // POSITION + CONFIGURATION REGULARIZATION --> TODO


  // /////////////////////////
  // // POSITION + ORIENTATION  --> NOPE
  // // pin::SE3 e = x_r.inverse() * T_o_e_m;
  // pin::SE3 e = T_o_e_m.inverse() * x_r;
  // pin::Motion de = nu_o_e_m - dx_r;
  // Vector6d ddx_d = ddx_r - Kd_*de - Kp_*pin::log6(e);

  // // Create and solve least square problem to get desired joint acceleration
  // Eigen::Matrix<double, 6, 6> Jlog; pin::Jlog6(e, Jlog);
  // Eigen::MatrixXd A =  Jlog * J_pin;
  // Eigen::VectorXd b = ddx_d - dJ_pin * dq_m;
  // Vector7d ddq_d = A.colPivHouseholderQr().solve(b);
  // /////////////////////////
  

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

  ROS_INFO_STREAM("TaskSpaceIDController::compute_sinusoid_pose_reference pose_0: \n" << pose_0);
  ROS_INFO_STREAM("TaskSpaceIDController::compute_sinusoid_pose_reference nu: \n" << nu.transpose());
  ROS_INFO_STREAM("TaskSpaceIDController::compute_sinusoid_pose_reference pin::exp6(nu): \n" << pin::exp6(nu));

  x_r = pose_0 * pin::exp6(nu);
  ROS_INFO_STREAM("TaskSpaceIDController::compute_sinusoid_pose_reference x_r: \n" << x_r);
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
  // TODO: 
}

}  // namespace panda_torque_mpc

PLUGINLIB_EXPORT_CLASS(panda_torque_mpc::TaskSpaceIDController,
                       controller_interface::ControllerBase)
