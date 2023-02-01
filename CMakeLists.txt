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
  visualization_msgs
)

find_package(Eigen3 REQUIRED)
find_package(Franka 0.9.0 QUIET)
if(NOT Franka_FOUND)
  find_package(Franka 0.8.0 REQUIRED)
endif()
find_package(pinocchio REQUIRED)

add_message_files(FILES
  JointValuesComparison.msg
)

generate_messages()

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
    hardware_interface
    joint_limits_interface
    tf
    tf_conversions
    message_runtime
    pluginlib
    realtime_tools
    roscpp
    urdf
    visualization_msgs
  DEPENDS Franka
)


add_library(panda_torque_mpc
  src/model_pinocchio_vs_franka_controller.cpp
  src/joint_space_ID_controller.cpp
  src/task_space_ID_controller.cpp
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

target_include_directories(panda_torque_mpc PUBLIC ${PINOCCHIO_INCLUDE_DIRS})
target_link_libraries(panda_torque_mpc PUBLIC ${PINOCCHIO_LIBRARIES}  )