cmake_minimum_required(VERSION 3.10)
project(panda_torque_mpc)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  urdf
  pluginlib
  controller_interface
  hardware_interface
  joint_limits_interface
  realtime_tools
  dynamic_reconfigure
  eigen_conversions
  franka_hw
  franka_gripper
  geometry_msgs
  visualization_msgs
  linear_feedback_controller_msgs
  message_generation
  tf
  tf_conversions
  )

find_package(Eigen3 REQUIRED)
find_package(Franka 0.9.0 QUIET)
if(NOT Franka_FOUND)
  find_package(Franka 0.8.0 REQUIRED)
endif()
find_package(pinocchio REQUIRED)
find_package(tsid REQUIRED)
find_package(crocoddyl REQUIRED)

add_message_files(
  FILES
  JointValuesComparison.msg
  TaskPoseComparison.msg
  TaskTwistComparison.msg
  PoseTaskGoal.msg
)

generate_messages(DEPENDENCIES geometry_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    # rospy
    urdf
    pluginlib
    controller_interface
    hardware_interface
    joint_limits_interface
    realtime_tools
    dynamic_reconfigure
    eigen_conversions
    franka_hw
    franka_gripper
    geometry_msgs
    visualization_msgs
    linear_feedback_controller_msgs
    # message_generation
    tf
    tf_conversions
  DEPENDS Franka pinocchio tsid crocoddyl
)

###############
# Utils library
###############

add_library(panda_torque_mpc_utils
  src/crocoddyl_reaching.cpp
)

target_include_directories(panda_torque_mpc_utils PUBLIC
  include
)

target_link_libraries(panda_torque_mpc_utils PUBLIC
  Eigen3::Eigen
  pinocchio::pinocchio 
  pinocchio::pinocchio_headers
  crocoddyl::crocoddyl
)

#######################
# ROS control instances
#######################

add_library(panda_torque_mpc
  src/ctrl_model_pinocchio_vs_franka.cpp
  src/ctrl_log_update_dt.cpp
  src/ctrl_playback_pd_plus.cpp
  src/ctrl_joint_space_ID.cpp
  src/ctrl_task_space_ID.cpp
  src/ctrl_mpc_croco.cpp
  src/ctrl_mpc_linearized.cpp
)

target_include_directories(panda_torque_mpc SYSTEM PUBLIC
  ${catkin_INCLUDE_DIRS}
  ${Franka_INCLUDE_DIRS}
  include
)

target_link_libraries(panda_torque_mpc PUBLIC 
  ${catkin_LIBRARIES}
  ${Franka_LIBRARIES}
  panda_torque_mpc_utils
  Eigen3::Eigen
  pinocchio::pinocchio 
  pinocchio::pinocchio_headers
  tsid::tsid
)

# Make sure that the messages are built before this target
add_dependencies(panda_torque_mpc panda_torque_mpc_generate_messages_cpp)

##############################
# Motion server crocoddyl node
##############################
add_executable(croccodyl_motion_server_node src/croccodyl_motion_server_node.cpp)

target_link_libraries(croccodyl_motion_server_node PUBLIC
  ${catkin_LIBRARIES}
  panda_torque_mpc_utils
  pinocchio::pinocchio 
  pinocchio::pinocchio_headers
)

target_include_directories(croccodyl_motion_server_node SYSTEM PUBLIC
  ${catkin_INCLUDE_DIRS}
  include
)

# Make sure that the messages are built before this target
add_dependencies(croccodyl_motion_server_node panda_torque_mpc_generate_messages_cpp)
