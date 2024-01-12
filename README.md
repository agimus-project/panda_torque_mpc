# panda_torque_mpc
------------------
Torque control MPC of a Panda manipulator with ROS 1 and roscontrol. 

# Build
## conda/mamba env fast setup
`mamba` is faster but you can use conda interchangeably.  
`conda create -n panda_control`
`conda activate panda_control`
`mamba env update --file environment.yaml`

## Other dependencies
In your catkin workspace `src` folder:  
`git clone git@github.com:loco-3d/linear-feedback-controller-msgs.git`

## Build catkin package
`CMAKE_BUILD_PARALLEL_LEVEL=4 catkin build panda_torque_mpc -DCMAKE_BUILD_TYPE=RELEASE`

# Launch
## Simulation (gazebo)
In two different shells:  

* `roslaunch franka_gazebo panda.launch arm_id:=panda`
* `roslaunch panda_torque_mpc sim_controllers.launch controller:=<controller-name>`

## Real
Set `PANDA_IP` env variable, e.g.:  
`export PANDA_IP=192.168.102.11` (CIIRC 6th floor)  
`export PANDA_IP=192.168.88.140` (CIIRC 4th floor)  
`export PANDA_IP=172.17.1.3`     (adream)  

* Bring robot to init position  
`roslaunch panda_torque_mpc move_to_start.launch robot_ip:=$PANDA_IP robot:=panda`
* Start one of the controllers
`roslaunch panda_torque_mpc real_controllers.launch controller:=<controller-name> robot_ip:=$PANDA_IP robot:=panda`

## Custom controllers
The parameters of each controller are defined in `config/controller_configs.yaml`. To run one of them in simulation or real, replace <controller-name> with:
* `ctrl_model_pinocchio_vs_franka`: print comparison of Rigid Body Dynamics computation between pinocchio+example-robot-data and libfranka
* `ctrl_log_update_dt`: logs ::update time and duration parameters in a csv file to investigate RT control
* `ctrl_playback_pd_plus`: reads a joint trajectory stored in csv files q.csv, v.csv, tau.csv and plays it back using PD+ 
* `ctrl_joint_space_ID`: follow joint trajectory reference using different flavors of joint space Inverse Dynamics 
* `ctrl_task_space_ID`: follow task space end-effector trajectory ($\mathbb{R}^3$ or SE(3)) 
* `ctrl_mpc_croco`: synchronously solving of OCP using crocoddyl and sending the first torque command -> (!) limited to very short horizons to avoid breaking real time constraint 
* `ctrl_mpc_linearized`: asynchronous execution a linearized control reference from OCP solver running in another node (crocoddyl_motion_server_node) using Ricatti gains -> very few computation, no update() skipped

<!-- ## Realsense T265 demo with TSID (launch in this order in different shells)
```bash
roslaunch realsense2_camera demo_t265.launch  
ROS_NAMESPACE=/ctrl_task_space_ID rosrun panda_torque_mpc pose_publisher.py  
roslaunch franka_gazebo panda.launch  
roslaunch panda_torque_mpc sim_controllers.launch controller:=ctrl_task_space_ID  
```

## Realsense T265 demo with asynchronous MPC (simu)
```bash
roslaunch realsense2_camera demo_t265.launch
ROS_NAMESPACE=/ctrl_mpc_linearized rosrun panda_torque_mpc pose_publisher.py
roslaunch franka_gazebo panda.launch
roslaunch panda_torque_mpc sim_controllers.launch controller:=ctrl_mpc_linearized record_joints:=true
ROS_NAMESPACE=/ctrl_mpc_linearized rosrun panda_torque_mpc crocoddyl_motion_server_node
``` -->

<!-- ## Example demos
### Realsense T265 demo with asynchronous MPC (real)
```bash
roslaunch realsense2_camera demo_t265.launch
ROS_NAMESPACE=/ctrl_mpc_linearized rosrun panda_torque_mpc pose_publisher.py
roslaunch panda_torque_mpc real_controllers.launch controller:=ctrl_mpc_linearized robot_ip:=$PANDA_IP robot:=panda
ROS_NAMESPACE=/ctrl_mpc_linearized rosrun panda_torque_mpc crocoddyl_motion_server_node
``` -->

### Impulse test with asynchronous MPC (simu)
```bash
roslaunch franka_gazebo panda.launch arm_id:=panda
roslaunch panda_torque_mpc sim_controllers.launch controller:=ctrl_mpc_linearized
ROS_NAMESPACE=/ctrl_mpc_linearized rosrun panda_torque_mpc crocoddyl_motion_server_node
ROS_NAMESPACE=/ctrl_mpc_linearized rosrun panda_torque_mpc pose_publisher_simulated_object_mvt.py
```

### Impulse test with asynchronous MPC (real)
```bash
roslaunch panda_torque_mpc real_controllers.launch controller:=ctrl_mpc_linearized robot_ip:=$PANDA_IP robot:=panda
ROS_NAMESPACE=/ctrl_mpc_linearized rosrun panda_torque_mpc crocoddyl_motion_server_node
ROS_NAMESPACE=/ctrl_mpc_linearized rosrun panda_torque_mpc pose_publisher_simulated_object_mvt.py
```

### Follow sinusoid end-effector reference
Same as "Impulse test with asynchronous MPC", replace last command by  
`ROS_NAMESPACE=/ctrl_mpc_linearized rosrun panda_torque_mpc pose_publisher.py --compute_local_sinusoid`

### Realsense VISUAL SERVOING demo with asynchronous MPC (real)
```bash
roslaunch realsense2_camera rs_camera.launch
ROS_NAMESPACE=/ctrl_mpc_linearized rosrun panda_torque_mpc pose_publisher.py --visual_servoing
roslaunch panda_torque_mpc real_controllers.launch controller:=ctrl_mpc_linearized robot_ip:=$PANDA_IP robot:=panda
ROS_NAMESPACE=/ctrl_mpc_linearized rosrun panda_torque_mpc crocoddyl_motion_server_node
```

### Realsense VISUAL SERVOING demo with asynchronous MPC (real,apriltag)
```bash
roslaunch realsense2_camera rs_camera.launch
roslaunch apriltag_ros continuous_detection.launch  # check the tag id/tag size etc. -> TODO: document
roslaunch panda_torque_mpc real_controllers.launch controller:=ctrl_mpc_linearized robot_ip:=$PANDA_IP robot:=panda
ROS_NAMESPACE=/ctrl_mpc_linearized rosrun panda_torque_mpc crocoddyl_motion_server_node
```

# TODOLIST
* Double check if `initialized` topic is streamed when using the real controller (not likely) 
* Check if latest franka_ros fixes inverted torque measurements in gazebo sim
* Refactor log publishers -> LoggingExperiment class with RTpublishers?
