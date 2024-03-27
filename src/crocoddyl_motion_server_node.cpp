#include <boost/smart_ptr/shared_ptr.hpp>
#include <string>
#include <cassert>
 
// Use (void) to silence unused warnings.
#define assertm(exp, msg) assert(((void)msg, exp))

#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/multibody/fcl.hpp>

#include <hpp/fcl/collision_object.h>
#include <hpp/fcl/shape/geometric_shapes.h>


#include <linear_feedback_controller_msgs/Sensor.h>
#include <linear_feedback_controller_msgs/Control.h>
#include <linear_feedback_controller_msgs/eigen_conversions.hpp>


#include <realtime_tools/realtime_box.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "panda_torque_mpc/common.h"
#include "panda_torque_mpc/crocoddyl_reaching.h"

#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Duration.h"



namespace panda_torque_mpc
{
    namespace lfc_msgs = linear_feedback_controller_msgs;

    class CrocoMotionServer
    {
    public:
        CrocoMotionServer(ros::NodeHandle &nh,
                          std::string robot_sensors_topic_sub,
                          std::string control_topic_pub,
                          std::string absolute_pose_ref_topic_sub,
                          std::string motion_capture_pose_ref_topic_sub,
                          std::string pose_camera_object_topic_sub, 
                          std::string pose_object_rel_topic_sub, 
                          std::string cam_pose_viz_topic_pub,
                          std::string cam_pose_ref_viz_topic_pub,
                          std::string cam_pose_error_topic_pub,
                          std::string ee_pose_error_topic_pub,
                          std::string ocp_solve_time_topic_pub
                          )
        {
            bool params_success = true;
            ///////////////////
            // Load parameters
            ///////////////////
            std::string arm_id;
            params_success = get_param_error_tpl<std::string>(nh, arm_id, "arm_id") && params_success;

            // Croco params
            int nb_shooting_nodes, nb_iterations_max, max_qp_iter;
            double dt_ocp,solver_termination_tolerance,qp_termination_tol_abs , qp_termination_tol_rel, w_frame_running, w_frame_terminal, w_frame_vel_running, w_frame_vel_terminal, w_x_reg_running, w_x_reg_terminal, w_u_reg_running, publish_frequency, w_slope, w_cut;
            std::vector<double> diag_frame_vel, diag_q_reg_running, diag_v_reg_running, diag_u_reg_running, armature;
            std::vector<double> pose_e_c, pose_c_o_ref, pose_target1, pose_target2;  // px,py,pz, qx,qy,qz,qw
            std::vector<pin::SE3> pose_targets;
            bool reference_is_placement;

            params_success = get_param_error_tpl<int>(nh, nb_shooting_nodes, "nb_shooting_nodes") && params_success;
            params_success = get_param_error_tpl<double>(nh, dt_ocp, "dt_ocp") && params_success;
            params_success = get_param_error_tpl<double>(nh, solver_termination_tolerance, "solver_termination_tolerance") && params_success;
            params_success = get_param_error_tpl<double>(nh, qp_termination_tol_abs, "qp_termination_tol_abs") && params_success;
            params_success = get_param_error_tpl<double>(nh, qp_termination_tol_rel, "qp_termination_tol_rel") && params_success;
            params_success = get_param_error_tpl<int>(nh, nb_iterations_max, "nb_iterations_max") && params_success;
            params_success = get_param_error_tpl<int>(nh, max_qp_iter, "max_qp_iter") && params_success;
            params_success = get_param_error_tpl<bool>(nh, reference_is_placement, "reference_is_placement") && params_success;
            params_success = get_param_error_tpl<bool>(nh, keep_original_ee_rotation_, "keep_original_ee_rotation") && params_success;
            params_success = get_param_error_tpl<double>(nh, w_frame_running,  "w_frame_running") && params_success;
            params_success = get_param_error_tpl<double>(nh, w_frame_terminal, "w_frame_terminal") && params_success;
            params_success = get_param_error_tpl<double>(nh, w_frame_vel_running,  "w_frame_vel_running") && params_success;
            params_success = get_param_error_tpl<double>(nh, w_frame_vel_terminal, "w_frame_vel_terminal") && params_success;
            params_success = get_param_error_tpl<double>(nh, w_x_reg_running,  "w_x_reg_running") && params_success;
            params_success = get_param_error_tpl<double>(nh, w_x_reg_terminal, "w_x_reg_terminal") && params_success;
            params_success = get_param_error_tpl<double>(nh, w_u_reg_running,  "w_u_reg_running") && params_success;
            params_success = get_param_error_tpl<double>(nh, publish_frequency,  "publish_frequency") && params_success;
            params_success = get_param_error_tpl<double>(nh, w_slope,  "w_slope") && params_success;
            params_success = get_param_error_tpl<double>(nh, w_cut,  "w_cut") && params_success;

            params_success = get_param_error_tpl<std::vector<double>>(nh, diag_frame_vel, "diag_frame_vel",
                                                                      [](std::vector<double> v)
                                                                      { return v.size() == 6; }) && params_success;
            params_success = get_param_error_tpl<std::vector<double>>(nh, diag_q_reg_running, "diag_q_reg_running",
                                                                      [](std::vector<double> v)
                                                                      { return v.size() == 7; }) && params_success;
            params_success = get_param_error_tpl<std::vector<double>>(nh, diag_v_reg_running, "diag_v_reg_running",
                                                                      [](std::vector<double> v)
                                                                      { return v.size() == 7; }) && params_success;
            params_success = get_param_error_tpl<std::vector<double>>(nh, diag_u_reg_running, "diag_u_reg_running",
                                                                      [](std::vector<double> v)
                                                                      { return v.size() == 7; }) && params_success;
            params_success = get_param_error_tpl<std::vector<double>>(nh, armature, "armature",
                                                                      [](std::vector<double> v)
                                                                      { return v.size() == 7; }) && params_success;

            params_success = get_param_error_tpl<std::vector<double>>(nh, pose_e_c, "pose_e_c",
                                                                      [](std::vector<double> v)
                                                                      { return v.size() == 7; }) && params_success;
            params_success = get_param_error_tpl<std::vector<double>>(nh, pose_c_o_ref, "pose_c_o_ref",
                                                                      [](std::vector<double> v)
                                                                      { return v.size() == 7; }) && params_success;
            params_success = get_param_error_tpl<std::vector<double>>(nh, pose_target1, "pose_target1",
                                                                      [](std::vector<double> v)
                                                                      { return v.size() == 7; }) && params_success;
            params_success = get_param_error_tpl<std::vector<double>>(nh, pose_target2, "pose_target2",
                                                                      [](std::vector<double> v)
                                                                      { return v.size() == 7; }) && params_success;                                                          
            params_success = get_param_error_tpl<std::string>(nh, ee_frame_name_, "ee_frame_name") && params_success;

            
            if (!params_success)
            {
                throw std::invalid_argument("CrocoMotionServer: check the your ROS parameters");
            }

            model_pin_ = loadPandaPinocchio();
            data_pin_ = pin::Data(model_pin_);

            // Creating the collision model
            std::string urdf_path = ros::package::getPath("panda_torque_mpc") + "/urdf/robot.urdf";

            // Building the GeometryModel
            auto collision_model = boost::make_shared<pinocchio::GeometryModel>();
            collision_model = loadPandaGeometryModel(model_pin_);

            double radius = 0.35/2.0;

            auto geometry = pinocchio::GeometryObject::CollisionGeometryPtr(new hpp::fcl::Sphere(radius));

            pinocchio::SE3 obstacle_pose(Eigen::Quaterniond (1.,0.,0.,0.), Eigen::Vector3d (0,0,0.825));
            // pinocchio::SE3 obstacle_pose;
            // obstacle_pose.setIdentity();
            // obstacle_pose.trans << 0., 0., 0.;

            pinocchio::GeometryObject obstacle("obstacle", 0,0, geometry, obstacle_pose);
            collision_model->addGeometryObject(obstacle);


            assertm(collision_model->getGeometryId("obstacle") < collision_model->geometryObjects.size(), "The index of the obstacle is not right.");
            assertm(collision_model->getGeometryId("panda_leftfinger_0") < collision_model->geometryObjects.size(), "The index of the panda_leftfinger_0 is not right.");
            assertm(collision_model->getGeometryId("panda_rightfinger_0") < collision_model->geometryObjects.size(), "The index of the panda_rightfinger_0 is not right.");

            //   Print out the placement of each collision geometry object

            collision_model->addCollisionPair(pinocchio::CollisionPair(collision_model->getGeometryId("obstacle"),
                collision_model->getGeometryId("panda_leftfinger_0")));

            collision_model->addCollisionPair(pinocchio::CollisionPair(collision_model->getGeometryId("obstacle"),
                collision_model->getGeometryId("panda_rightfinger_0")));

            collision_model->addCollisionPair(pinocchio::CollisionPair(collision_model->getGeometryId("obstacle"),
                collision_model->getGeometryId("panda_link7_sc_1")));

            collision_model->addCollisionPair(pinocchio::CollisionPair(collision_model->getGeometryId("obstacle"),
                collision_model->getGeometryId("panda_link7_sc_4")));
                        
            if ((model_pin_.nq != 7) || (model_pin_.name != "panda"))
            {
                ROS_ERROR_STREAM("Problem when loading the robot urdf");
                throw std::invalid_argument("CrocoMotionServer: Problem with the loaded robot model");
            } 

            // Define corresponding frame id for pinocchio and Franka (see ctrl_model_pinocchio_vs_franka)
            ee_frame_id_ = model_pin_.getFrameId(ee_frame_name_);

            // add targets pose to vector of targets pose
            pose_targets.push_back(panda_torque_mpc::XYZQUATToSE3(pose_target1));
            pose_targets.push_back(panda_torque_mpc::XYZQUATToSE3(pose_target2));
            
            /////////////////////////////////////////////////
            //                MPC CONFIG                   //
            /////////////////////////////////////////////////
            config_croco_.T = nb_shooting_nodes;
            config_croco_.dt_ocp = dt_ocp;
            config_croco_.solver_termination_tolerance= solver_termination_tolerance;
            config_croco_.qp_termination_tol_abs= qp_termination_tol_abs;
            config_croco_.qp_termination_tol_rel= qp_termination_tol_rel;
            config_croco_.nb_iterations_max = nb_iterations_max;
            config_croco_.max_qp_iter = max_qp_iter;
            config_croco_.ee_frame_name = ee_frame_name_;
            config_croco_.reference_is_placement = reference_is_placement;
            config_croco_.w_frame_running =  w_frame_running;
            config_croco_.w_frame_terminal = w_frame_terminal;
            config_croco_.w_frame_vel_running =  w_frame_vel_running;
            config_croco_.w_frame_vel_terminal = w_frame_vel_terminal;
            config_croco_.diag_frame_vel = Eigen::Map<Eigen::Matrix<double, 6, 1>>(diag_frame_vel.data()) ;
            config_croco_.w_x_reg_running =  w_x_reg_running;
            config_croco_.w_x_reg_terminal = w_x_reg_terminal;
            config_croco_.diag_q_reg_running = Eigen::Map<Eigen::Matrix<double, 7, 1>>(diag_q_reg_running.data()) ;
            config_croco_.diag_v_reg_running = Eigen::Map<Eigen::Matrix<double, 7, 1>>(diag_v_reg_running.data());
            config_croco_.w_u_reg_running = w_u_reg_running;
            config_croco_.diag_u_reg_running = Eigen::Map<Eigen::Matrix<double, 7, 1>>(diag_u_reg_running.data());
            config_croco_.armature = Eigen::Map<Eigen::Matrix<double, 7, 1>>(armature.data());

            TargetsConfig targ_config_;
            targ_config_.pose_targets = pose_targets;
            targ_config_.publish_frequency = publish_frequency;
            targ_config_.nb_target = pose_targets.size();
            targ_config_.cycle_duration = targ_config_.nb_target/publish_frequency;
            targ_config_.cycle_duration_2 = targ_config_.cycle_duration /2;
            targ_config_.cycle_nb_nodes = targ_config_.cycle_duration / dt_ocp;
            targ_config_.w_slope = w_slope;
            targ_config_.w_cut = w_cut;

            // croco_reaching_ = CrocoddylReaching(model_pin_ ,config_croco_);
            croco_reaching_ = CrocoddylReaching(model_pin_, collision_model ,config_croco_, targ_config_);
            /////////////////////////////////////////////////

            // Publishers
            control_pub_ = nh.advertise<lfc_msgs::Control>(control_topic_pub, 1);
            cam_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>(cam_pose_viz_topic_pub, 1);
            cam_pose_ref_pub_ = nh.advertise<geometry_msgs::PoseStamped>(cam_pose_ref_viz_topic_pub, 1);
            cam_pose_error_pub_ = nh.advertise<geometry_msgs::PoseStamped>(cam_pose_error_topic_pub, 1);
            ee_pose_error_pub_ = nh.advertise<geometry_msgs::PoseStamped>(ee_pose_error_topic_pub, 1);
            ocp_solve_time_pub_ = nh.advertise<std_msgs::Duration>(ocp_solve_time_topic_pub, 1);
            
            // Subscribers
            sensor_sub_ = nh.subscribe(robot_sensors_topic_sub, 10, &CrocoMotionServer::callback_robot_state, this);
            pose_absolute_sub_ = nh.subscribe(absolute_pose_ref_topic_sub, 10, &CrocoMotionServer::callback_pose_absolute, this);
            pose_mocap_sub_ = nh.subscribe(motion_capture_pose_ref_topic_sub, 10, &CrocoMotionServer::callback_pose_mocap, this);
            pose_camera_object_sub_ = nh.subscribe(pose_camera_object_topic_sub, 10, &CrocoMotionServer::callback_pose_camera_object, this);
            pose_body_object_rel_sub_ = nh.subscribe(pose_object_rel_topic_sub, 10, &CrocoMotionServer::callback_pose_object0_object, this);

            // State machine variables
            first_robot_sensor_msg_received_ = false;
            first_pose_ref_msg_received_ = false;
            first_solve_ = true;

            T_e_c_ = XYZQUATToSE3(pose_e_c);
            T_c_e_ = T_e_c_.inverse();
            T_c_o_ref_ = XYZQUATToSE3(pose_c_o_ref);
            T_o_c_ref_ = T_c_o_ref_.inverse();
        }

        void callback_pose_absolute(const geometry_msgs::PoseStamped &msg)
        {
            /**
             * Callback using the pose as a reference in robot base frame 
             */
            pin::SE3 T_b_e_ref = posemsg2SE3(msg.pose);
            T_b_e_ref_rtbox_ = T_b_e_ref;
            if (!first_pose_ref_msg_received_)
            {
                first_pose_ref_msg_received_ = true;
            }


            // ------- LOGS ----------- //
            // Reference tracking error
            pin::SE3 T_b_e; T_b_e = T_b_e_rtbox_;
            pin::SE3 T_b_e_error = T_b_e_ref.inverse() * T_b_e;
            publish_SE3_posestamped(ee_pose_error_pub_, T_b_e_error, msg.header);
        }

        void callback_pose_mocap(const geometry_msgs::PoseStamped &msg)
        {
            /**
             * Callback for motion capture demo (e.g. with t265), not part of the visual servoing experiments.
             * 
             * If the first sensor state of the robot has not yet been received, no need to process the pose ref
             */

            if (!first_robot_sensor_msg_received_)
            {
                return;
            }

            pin::SE3 T_w_t = posemsg2SE3(msg.pose);

            // Cf ctrl_task_space_ID for instance for why this choice
            // TRANSLATION ref
            pin::SE3 T_e0_e = pin::SE3::Identity();
            auto R_e0_b = T_b_e0_.rotation().transpose();
            T_e0_e.translation() = R_e0_b * (T_w_t.translation() - T_w_t0_.translation());

            // // ROTATION ref  --> PBE for now
            // Eigen::Matrix3d R_w_t0 = T_w_t0_.rotation();
            // Eigen::Matrix3d R_e0_t0 = R_e0_b* R_w_t0;
            // Eigen::Matrix3d R_t0_t = R_w_t0.transpose() * T_w_t.rotation();
            // T_e0_e.rotation() = R_e0_t0 * R_t0_t;

            // Set reference pose
            // compose initial pose with relative/local transform
            pin::SE3 T_b_e_ref = T_b_e0_ * T_e0_e;
            T_b_e_ref_rtbox_ = T_b_e_ref;

            if (!first_pose_ref_msg_received_)
            {
                T_w_t0_ = T_w_t;
                first_pose_ref_msg_received_ = true;
            }

            // ------- LOGS ----------- //
            // Reference tracking error
            pin::SE3 T_b_e; T_b_e = T_b_e_rtbox_;
            pin::SE3 T_b_e_error = T_b_e_ref.inverse() * T_b_e;
            publish_SE3_posestamped(ee_pose_error_pub_, T_b_e_error, msg.header);
        }


        void callback_pose_camera_object(const geometry_msgs::PoseStamped &msg_pose_c_o)
        {
            /**
             * If the first sensor state of the robot has not yet been received, no need to process the pose ref
             */
            if (!first_robot_sensor_msg_received_)
            {
                return;
            }

            pin::SE3 T_c_o_meas = posemsg2SE3(msg_pose_c_o.pose);
            pin::SE3 T_o_c_meas = T_c_o_meas.inverse();

            pin::SE3 T_b_e; T_b_e = T_b_e_rtbox_;

            pin::SE3 T_b_c_ref = T_b_e * T_e_c_ * T_c_o_meas * T_o_c_ref_;
            pin::SE3 T_b_e_ref = T_b_c_ref * T_c_e_;

            // RT safe setting
            T_b_e_ref_rtbox_ = T_b_e_ref;

            if (!first_pose_ref_msg_received_)
            {
                first_pose_ref_msg_received_ = true;
            }

            // ------- LOGS ----------- //

            // Send message with current camera pose from current kinematics
            pin::SE3 T_b_c = T_b_e * T_e_c_;
            publish_SE3_posestamped(cam_pose_pub_, T_b_c, msg_pose_c_o.header);

            // Send message with reference camera pose
            publish_SE3_posestamped(cam_pose_ref_pub_, T_b_c_ref, msg_pose_c_o.header);

            // Reference tracking error
            pin::SE3 T_b_c_error = T_b_c_ref.inverse() * T_b_c;
            publish_SE3_posestamped(cam_pose_error_pub_, T_b_c_error, msg_pose_c_o.header);
        }

        void callback_pose_object0_object(const geometry_msgs::PoseStamped &msg_pose_o0_o)
        {
            
            /**
             * Idea: simulate a visual servoing task by updating an initial object pose computed from 
             * - T_c_o_ref_
             * - T_b_e0_ (set when first robot sensor message received)
             * - T_o0_o contained in msg_pose_o0_o, should start with identity and then send relative poses
             * from initial virtual robot 
             * 
            */

            pin::SE3 T_b_e; T_b_e = T_b_e_rtbox_;

            if (!first_pose_ref_msg_received_)
            {
                T_b_o_init_ = T_b_e * T_e_c_ * T_c_o_ref_;
            }

            pin::SE3 T_o0_o_meas = posemsg2SE3(msg_pose_o0_o.pose);

            // Simulate change in object pose relative to origin
            pin::SE3 T_b_o = T_b_o_init_ * T_o0_o_meas;

            // Recover reference base pose so that T_o_c_ref_ is maintained
            pin::SE3 T_b_c_ref = T_b_o * T_o_c_ref_;
            pin::SE3 T_b_e_ref = T_b_c_ref * T_c_e_;
            
            // RT safe setting
            T_b_e_ref_rtbox_ = T_b_e_ref;

            if (!first_pose_ref_msg_received_)
            {
                first_pose_ref_msg_received_ = true;
            }

            // ------- LOGS ----------- //

            // Send message with current camera pose from current kinematics
            pin::SE3 T_b_c = T_b_e * T_e_c_;
            publish_SE3_posestamped(cam_pose_pub_, T_b_c, msg_pose_o0_o.header);

            // Send message with reference camera pose
            publish_SE3_posestamped(cam_pose_ref_pub_, T_b_c_ref, msg_pose_o0_o.header);

            pin::SE3 T_b_c_error = T_b_c_ref.inverse() * T_b_c;
            publish_SE3_posestamped(cam_pose_error_pub_, T_b_c_error, msg_pose_o0_o.header);
        }


        void callback_robot_state(const lfc_msgs::Sensor &sensor_msg)
        {
            // Recover latest robot state from the sensor msg
            t_sensor_ = sensor_msg.header.stamp;
            lfc_msgs::Eigen::Sensor sensor_eig;
            lfc_msgs::sensorMsgToEigen(sensor_msg, sensor_eig);
            Eigen::Matrix<double, 14, 1> current_x;
            current_x << sensor_eig.joint_state.position, sensor_eig.joint_state.velocity;
            current_x_rtbox_ = current_x;

            pin::forwardKinematics(model_pin_, data_pin_, sensor_eig.joint_state.position);
            pin::updateFramePlacements(model_pin_, data_pin_);

            pin::SE3 T_b_e = data_pin_.oMf[ee_frame_id_];
            T_b_e_rtbox_ = T_b_e;

            if (!first_robot_sensor_msg_received_)
            {
                T_b_e0_ = T_b_e;

                q_init_rtbox_ = sensor_eig.joint_state.position;
                first_robot_sensor_msg_received_ = true;
            }
        }

        void solve_and_send()
        {
            // Do nothing if no pose reference and sensor state has been received
            if (!(first_robot_sensor_msg_received_ && first_pose_ref_msg_received_))
            {
                croco_reaching_.simulation_time.tic();
                return;
            }

            // Retrieve end effector reference/current in thread-safe way
            pin::SE3 T_b_e_ref; T_b_e_ref = T_b_e_ref_rtbox_;
            pin::SE3 T_b_e; T_b_e = T_b_e_rtbox_;

            // Retrieve initial configuration in thread-safe way
            Vector7d q_init; q_init = q_init_rtbox_;
            Eigen::Matrix<double,14,1> x_init; x_init << q_init, Vector7d::Zero();  // Fix zero velocity as reference

            // Retrieve current state in a thread-safe way
            Eigen::Matrix<double, 14, 1> current_x; current_x = current_x_rtbox_;
            Vector7d q = current_x.head(model_pin_.nq);
            Vector7d v = current_x.tail(model_pin_.nv);

            // State/control trajectories
            std::vector<Eigen::Matrix<double, -1, 1>> xs_init;
            std::vector<Eigen::Matrix<double, -1, 1>> us_init;
            if (first_solve_)
            {
                // Warm start with gravity compensation control term
                Vector7d tau_grav = pin::computeGeneralizedGravity(model_pin_, data_pin_, q);

                // if first occurence, use a sensible prior (no movement and gravity compensation)
                xs_init.reserve(config_croco_.T);
                us_init.reserve(config_croco_.T);
                for (int i = 0; i < config_croco_.T; i++)
                {
                    xs_init.push_back(current_x);
                    us_init.push_back(tau_grav);
                }
                xs_init.push_back(current_x);

                first_solve_ = false;
            }
            else
            {
                // Warm start with previous solution shifted
                xs_init = croco_reaching_.ocp_->get_xs();
                us_init = croco_reaching_.ocp_->get_us();
                
                /**
                 * Shift trajectory by 1 node <==> config_croco_.dt_ocp
                 * !!! HYP: config_croco_.dt_ocp == 1/freq_solve
                */
                // TODO: check if putting current measurement in initial guess state traj makes sense
                // xs_init.insert(std::begin(xs_init), current_x);
                // xs_init.insert(std::begin(xs_init), xs_init[0]);
                // xs_init.erase(std::end(xs_init) - 1);
                // us_init.insert(std::begin(us_init), us_init[0]);
                // us_init.erase(std::end(us_init) - 1);
            }

            // Set the fixed initial state for the OCP state trajectory using current measurements
            croco_reaching_.ocp_->get_problem()->set_x0(current_x);

            // Deactivating reaching task would requires to re-equilibrate the OCP weights
            // -> easier to track last known reference active
            bool reaching_task_is_active = true;
            if (config_croco_.reference_is_placement)
            {
                croco_reaching_.set_ee_ref_placement(T_b_e_ref, reaching_task_is_active, 1.0);
            }
            else
            {
                croco_reaching_.set_ee_ref_translation(T_b_e_ref.translation(), reaching_task_is_active);
            }
            croco_reaching_.set_posture_ref(x_init);

            TicTac tt_solve;
            bool ok = croco_reaching_.solve(xs_init, us_init);
            const auto duration = tt_solve.tac();
            std::cout << std::setprecision(9) << "n_iter, dt_solve (ms): " << croco_reaching_.ocp_->get_iter() << ", " << duration << std::endl;
            std_msgs::Duration time;
            time.data = ros::Duration(duration * 0.001);
            ocp_solve_time_pub_.publish(time);
            // if problem not ready or no good solution, don't send a solution
            if (!ok) return;
            //////////////////////////////////////

            // Fill and send control message
            lfc_msgs::Eigen::Control ctrl_eig;
            ctrl_eig.initial_state.joint_state.position = q;
            ctrl_eig.initial_state.joint_state.velocity = v;
            ctrl_eig.feedforward = croco_reaching_.get_tau_ff();
            ctrl_eig.feedback_gain = croco_reaching_.get_ricatti_mat();
            lfc_msgs::Control ctrl_msg;
            lfc_msgs::controlEigenToMsg(ctrl_eig, ctrl_msg);
            control_pub_.publish(ctrl_msg);
        }

        // sensor callback
        ros::Time t_sensor_;
        Vector7d q_init_rtbox_;
        Eigen::Matrix<double, 14, 1> current_x_rtbox_;
        pin::SE3 T_b_e0_;
        pin::SE3 T_b_e_rtbox_;

        // Visual servoing
        // Camera calibration
        pin::SE3 T_e_c_;
        pin::SE3 T_c_e_;

        // Pose reference
        pin::SE3 T_o_c_ref_;
        pin::SE3 T_c_o_ref_;

        // pose ref callback
        pin::SE3 T_w_t0_;
        pin::SE3 T_b_e_ref_rtbox_;

        // Solve state machine
        bool first_solve_;
        bool first_pose_ref_msg_received_;
        bool first_robot_sensor_msg_received_;

        // Pinocchio objects
        pin::Model model_pin_;
        pin::Data data_pin_;
        std::string ee_frame_name_;
        pin::FrameIndex ee_frame_id_;

        // MPC formulation
        CrocoddylReaching croco_reaching_;
        CrocoddylConfig config_croco_;
        double high_dist_ = 0.2;
        double low_dist_ = 0.03;
        double min_scaling_ = 0.1;
        double max_scaling_ = 1.0;

        // Publisher of commands
        ros::Publisher control_pub_;
        ros::Publisher cam_pose_pub_;
        ros::Publisher cam_pose_ref_pub_;
        ros::Publisher cam_pose_error_pub_;
        ros::Publisher ee_pose_error_pub_;
        ros::Publisher ocp_solve_time_pub_;

        // Subscriber to robot sensor from linearized ctrl
        ros::Subscriber sensor_sub_;

        // Subscriber to pose reference topic
        ros::Subscriber pose_absolute_sub_;
        ros::Subscriber pose_mocap_sub_;
        ros::Subscriber pose_camera_object_sub_;
        ros::Subscriber pose_body_object_rel_sub_;

        // Simulated object
        pin::SE3 T_b_o_init_;

        // DEMO mode
        bool keep_original_ee_rotation_ = false;

};

} // namespace panda_torque_mpc

int main(int argc, char **argv)
{

    ros::init(argc, argv, "crocoddyl_motion_server_node");

    ros::NodeHandle nh;

    std::string robot_sensors_topic_sub = "robot_sensors";
    std::string control_topic_pub = "motion_server_control";

    std::string absolute_pose_ref_topic_sub = "absolute_pose_ref";  // ABSOLUTE REFERENCE DEMO
    std::string motion_capture_pose_ref_topic_sub = "motion_capture_pose_ref";  // MOCAP DEMO
    std::string pose_camera_object_topic_sub = "pose_camera_object";  // VISUAL SERVOING DEMO 
    std::string pose_object_rel_topic_sub = "pose_object_rel";  // SIMULATION OF VIRTUAL OBJECT
    std::string cam_pose_viz_topic_pub = "cam_pose_viz";
    std::string cam_pose_ref_viz_topic_pub = "cam_pose_ref_viz";
    std::string cam_pose_error_topic_pub = "cam_pose_error";
    std::string ee_pose_error_topic_pub = "ee_pose_error";
    std::string ocp_solve_time_topic_pub = "ocp_solve_time";

    auto motion_server = panda_torque_mpc::CrocoMotionServer(
                            nh, 
                            robot_sensors_topic_sub,
                            control_topic_pub,
                            absolute_pose_ref_topic_sub,
                            motion_capture_pose_ref_topic_sub,
                            pose_camera_object_topic_sub,
                            pose_object_rel_topic_sub,
                            cam_pose_viz_topic_pub,
                            cam_pose_ref_viz_topic_pub,
                            cam_pose_error_topic_pub,
                            ee_pose_error_topic_pub,
                            ocp_solve_time_topic_pub
                            );

    int freq_solve;
    int success_read = panda_torque_mpc::get_param_error_tpl<int>(nh, freq_solve, "freq_solve");
    if (!success_read){
        throw std::invalid_argument("CrocoMotionServer: missing freq_solve parameter");
    }
    ros::Rate loop_rate(freq_solve);

    while (ros::ok())
    {
        // motion_server.retrieve_pose_ref_from_tf();
        motion_server.solve_and_send();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 1;
}