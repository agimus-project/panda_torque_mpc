cmake_minimum_required(VERSION 3.10)
project(panda_torque_mpc)


# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

find_package(catkin REQUIRED COMPONENTS
  controller_interface
  dynamic_reconfigure
  eigen_conversions
  franka_hw
  franka_gripper
  geometry_msgs
  visualization_msgs
  hardware_interface
  joint_limits_interface
  tf
  tf_conversions
  message_generation
  pluginlib
  realtime_tools
  roscpp
  rospy
  urdf
)

find_package(Eigen3 REQUIRED)
find_package(Franka 0.9.0 QUIET)
if(NOT Franka_FOUND)
  find_package(Franka 0.8.0 REQUIRED)
endif()
find_package(pinocchio REQUIRED)
find_package(tsid REQUIRED)

add_message_files(FILES
  JointValuesComparison.msg
  TaskPoseComparison.msg
  TaskTwistComparison.msg
)

generate_messages(DEPENDENCIES geometry_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES panda_torque_mpc
  CATKIN_DEPENDS
    controller_interface
    dynamic_reconfigure
    eigen_conversions
    franka_hw
    franka_gripper
    geometry_msgs
    visualization_msgs
    hardware_interface
    joint_limits_interface
    tf
    tf_conversions
    message_runtime
    pluginlib
    realtime_tools
    roscpp
    urdf
  DEPENDS Franka
)


add_library(panda_torque_mpc
  src/ctrl_model_pinocchio_vs_franka.cpp
  src/ctrl_log_update_dt.cpp
  src/ctrl_joint_space_ID.cpp
  src/ctrl_task_space_ID.cpp
  src/common.cpp
)

target_link_libraries(panda_torque_mpc PUBLIC
  ${Franka_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_include_directories(panda_torque_mpc SYSTEM PUBLIC
  ${Franka_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

target_include_directories(panda_torque_mpc PUBLIC
  include
)

target_link_libraries(panda_torque_mpc PUBLIC pinocchio::pinocchio pinocchio::pinocchio_headers)
target_link_libraries(panda_torque_mpc PUBLIC tsid::tsid)
