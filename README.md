# panda_torque_mpc
------------------
Torque control MPC of a Panda manipulator with ROS 1 and roscontrol. 

# Build
## conda/mamba env fast setup (Unsupported for now)
`mamba` is faster but you can use conda interchangeably.  
```
conda create -n panda_control
conda activate panda_control
mamba env update --file environment.yaml
```

## Build with ROS

This process assumes you have installed `python3-rosdep`, `python3-vcstool` and `python3-catkin-tools`.

> [!WARNING]  
> Building Pinocchio, HPP-FCL and Crocoddyl is very resource consuming!
>
> In case of 32 Gb of RAM it is suggested not to exceed 12 cores.
> If you have less memory, please change value in `-j` parameter of `catkin build`. 

```bash
# Inside ROS workspace
vcs import --recursive < src/panda_torque_mpc/panda_torque_mpc.repos src
rosdep update --rosdistro $ROS_DISTRO
rosdep install -y -i --from-paths src --rosdistro $ROS_DISTRO
catkin build -j12 --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_PYTHON_INTERFACE=OFF \
    -DBUILD_WITH_URDF_SUPPORT=ON \
    -DBUILD_WITH_COLLISION_SUPPORT=ON
```

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

* Bring robot to init position:  
`roslaunch panda_torque_mpc move_to_start.launch robot_ip:=$PANDA_IP robot:=panda`
* Start one of the controllers:  
`roslaunch panda_torque_mpc real_controllers.launch controller:=<controller-name> robot_ip:=$PANDA_IP robot:=panda`

## Custom controllers
The parameters of each controller are defined in `config/controller_configs.yaml`. To run one of them in simulation or real, replace <controller-name> with:
* `ctrl_model_pinocchio_vs_franka`: print comparison of Rigid Body Dynamics computation between pinocchio+example-robot-data and libfranka
* `ctrl_log_update_dt`: logs ::update time and duration parameters in a csv file to investigate RT control
* `ctrl_playback_pd_plus`: reads a joint trajectory stored in csv files q.csv, v.csv, tau.csv and plays it back using PD+ 
* `ctrl_joint_space_ID`: follow joint trajectory reference using different flavors of joint space Inverse Dynamics 
* `ctrl_task_space_ID`: follow task space end-effector trajectory ($\mathbb{R}^3$ or SE(3)) 
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
<!-- 
### Follow absolute end effector reference with asynchronous MPC (simu)
```bash
roslaunch panda_torque_mpc simulation.launch arm_id:=panda simulate_camera:=false
roslaunch panda_torque_mpc sim_controllers.launch controller:=ctrl_mpc_linearized
ROS_NAMESPACE=/ctrl_mpc_linearized rosrun panda_torque_mpc crocoddyl_motion_server_node
ROS_NAMESPACE=/ctrl_mpc_linearized roslaunch panda_torque_mpc pose_publisher.launch

``` -->


### Follow absolute end effector reference with asynchronous MPC (simu)
```bash
roslaunch panda_torque_mpc simulation.launch arm_id:=panda simulate_camera:=false
roslaunch panda_torque_mpc sim_controllers.launch controller:=ctrl_mpc_linearized
roslaunch panda_torque_mpc crocoddyl_motion_server_node.launch
ROS_NAMESPACE=/ctrl_mpc_linearized roslaunch panda_torque_mpc pose_publisher.launch

```

### Follow absolute end effector reference with asynchronous MPC (real)
```bash
roslaunch panda_torque_mpc real_controllers.launch controller:=ctrl_mpc_linearized robot_ip:=$PANDA_IP robot:=panda
roslaunch panda_torque_mpc crocoddyl_motion_server_node
ROS_NAMESPACE=/ctrl_mpc_linearized roslaunch panda_torque_mpc pose_publisher.launch
```

### Realsense VISUAL SERVOING demo with asynchronous MPC (real,apriltag)
First, check that `pose_e_c` and `pose_c_o_ref` have sensible values in `controller_config.yaml`
```bash
roslaunch realsense2_camera rs_camera.launch
roslaunch apriltag_ros continuous_detection.launch  # check the tag id/tag size etc.
roslaunch panda_torque_mpc real_controllers.launch controller:=ctrl_mpc_linearized robot_ip:=$PANDA_IP robot:=panda
roslaunch panda_torque_mpc crocoddyl_motion_server_node
```

# TODOLIST
* Double check if `initialized` topic is streamed when using the real controller (not likely) 
* Check if latest franka_ros fixes inverted torque measurements in gazebo sim
* Refactor log publishers -> LoggingExperiment class with RTpublishers?
* stream `pose_c_o_ref` from outside source
