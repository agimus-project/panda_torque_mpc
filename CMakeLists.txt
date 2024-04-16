cmake_minimum_required(VERSION 3.10)
project(panda_torque_mpc)
set(CMAKE_CXX_STANDARD 14)


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
find_package(crocoddyl REQUIRED)
find_package(mim_solvers REQUIRED)
find_package(colmpc REQUIRED)
find_package(tsid REQUIRED)

find_package(example-robot-data REQUIRED)


add_message_files(
  FILES
  JointValuesComparison.msg
  TaskPoseComparison.msg
  TaskTwistComparison.msg
)

generate_messages(DEPENDENCIES geometry_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
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

target_include_directories(panda_torque_mpc_utils SYSTEM PUBLIC
  ${catkin_INCLUDE_DIRS}
  )
target_include_directories(panda_torque_mpc_utils PUBLIC
  include
)

target_link_libraries(panda_torque_mpc_utils PUBLIC
  Eigen3::Eigen
  pinocchio::pinocchio
  pinocchio::pinocchio_headers
  crocoddyl::crocoddyl
  mim_solvers::mim_solvers
  colmpc::colmpc_cpp
  example-robot-data::example-robot-data
  tsid::tsid  
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
  src/ctrl_mpc_linearized.cpp
)

#message(FATAL_ERROR "dbg: ${catkin_INCLUDE_DIRS}")
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
  example-robot-data::example-robot-data
)

# Make sure that the messages are built before this target
add_dependencies(panda_torque_mpc panda_torque_mpc_generate_messages_cpp)

##############################
# Motion server crocoddyl node
##############################
add_executable(crocoddyl_motion_server_node src/crocoddyl_motion_server_node.cpp)

target_link_libraries(crocoddyl_motion_server_node PUBLIC
  ${catkin_LIBRARIES}
  panda_torque_mpc_utils
  pinocchio::pinocchio 
  pinocchio::pinocchio_headers
)

target_include_directories(crocoddyl_motion_server_node SYSTEM PUBLIC
  ${catkin_INCLUDE_DIRS}
  include
)


# Make sure that the messages are built before this target
add_dependencies(crocoddyl_motion_server_node panda_torque_mpc_generate_messages_cpp)

# Install launch files
install(FILES
  launch/franka_control.launch
  launch/move_to_start.launch
  launch/pose_publisher.launch
  launch/real_controllers.launch
  launch/record_joint_ctrl.launch
  launch/record_task_ctrl.launch
  launch/sim_controllers.launch
  launch/simulation.launch
  DESTINATION
  ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  )

# Install config files
install(FILES
  config/calibrated-params.yaml
  config/controller_configs.yaml
  config/franka_control_node.yaml
  config/pose_publisher_params.yaml
  config/start_joint_pose.yaml
  DESTINATION
  ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  )

# Install python nodes
install(PROGRAMS
  nodes/move_to_start.py
  nodes/pose_publisher.py
  DESTINATION
  ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
  )

# Install C++ nodes
install(TARGETS
  crocoddyl_motion_server_node
  DESTINATION
 ${CATKIN_PACKAGE_SHARE_DESTINATION} 
  )
  
# Install plugin description
install(FILES
  panda_torque_mpc_plugins.xml
  DESTINATION
  ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

# Install URDF files
install(FILES
  urdf/_d435.gazebo.xacro
  urdf/big_box.urdf
  urdf/demo.urdf.xacro
  urdf/franka_arm.xacro
  urdf/franka_hand.xacro
  urdf/franka_robot.xacro
  urdf/support.urdf.xacro
  urdf/utils.xacro
  urdf/robot.urdf
  DESTINATION
  ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
  )

# Install SRDF files
install(FILES
  srdf/big_box.srdf
  srdf/demo.srdf
  srdf/small_box.srdf
  DESTINATION
  ${CATKIN_PACKAGE_SHARE_DESTINATION}/srdf
  )

# Install meshes
install(FILES
  meshes/camera-support.stl
  DESTINATION
  ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes
)

FINDPYTHON()
set(PY_NAME "${PROJECT_NAME}_pywrap")
set(${PY_NAME}_INSTALL_DIR ${PYTHON_SITELIB}/${PY_NAME})
SEARCH_FOR_BOOST_PYTHON()

add_subdirectory(bindings)