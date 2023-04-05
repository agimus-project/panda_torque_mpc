# panda_torque_mpc
------------------  

# Building the project (tested with Ubuntu 20.04.05 - ROS noetic)
## conda/mamba env setup
`mamba` is faster but you can use conda interchangeably.

* Create environment:  
`conda create -n panda_control python=3.9`  
* Add package channels:  
`conda config --env --add channels conda-forge`    
`conda config --env --add channels robostack-staging`  
* Install ROS and other dependencies:  
`mamba install ros-noetic-desktop-full catkin_tools ros-noetic-combined-robot-hw pinocchio tsid`


## libfranka
Check [compatibility](https://frankaemika.github.io/docs/compatibility.html "FCI-libfranka compatibily matrix") of your Franka Control Interface version to determine which libfranka version to install.

At CIIRC, FCI version == 4.2.2  --> libfranka version >= 0.9.1 < 0.10.0. `robostack` conda channel provides 9.2 version which requires python 3.9 to be installed (hence `python=3.9` when creating the conda env).   
`mamba install ros-noetic-libfranka`

## Franka ROS
### From conda
Simpler but cannot make small modifications to the example controllers  
`mamba install ros-noetic-franka-ros`

### From source
In your catkin workspace src directory    
`git clone git@github.com:frankaemika/franka_ros.git`  
`catkin build franka_ros -DCMAKE_BUILD_TYPE=RELEASE`  

# Launch
## Simulation
In two different shells (change use_gripper according to which urdf model you use for control):  

* `roslaunch franka_gazebo panda.launch arm_id:=panda headless:=false use_gripper:=true`
* `roslaunch panda_torque_mpc sim_controllers.launch controller:=<controller-name>`

## Real
`robot_ip` and `load_gripper` arguments should be changed accordingly for each launch files

* Bring robot to init position  
`roslaunch panda_torque_mpc move_to_start.launch robot_ip:=192.168.102.11 load_gripper:=false robot:=panda`
* Start one of the custom controllers  
`roslaunch panda_torque_mpc real_controllers.launch controller:=<controller-name> robot_ip:=192.168.102.11 load_gripper:=false robot:=panda`

## Custom controllers
The parameters of each controller are defined in `config/controller_configs.yaml`. To run one of them in simulation or real, replace <controller-name> with:
* `ctrl_model_pinocchio_vs_franka`: compare Rigid Body Dynamics computation between pinocchio and libfranka
* `ctrl_log_update_dt`: logs ::update time and duration parameters in a csv file to investigate RT control
* `ctrl_playback_pd_plus`: reads a joint trajectory stored in csv files q.csv, v.csv, tau.csv and plays it back using PD+ 
* `ctrl_joint_space_ID`: follow joint trajectory reference using different flavors of joint space Inverse Dynamics 
* `ctrl_task_space_ID`: follow task space end-effector trajectory ($\mathbb{R}^3$ or SE(3)) 

## Realsense T265 demo (launch in this order)
`roslaunch realsense2_camera demo_t265.launch`  
`ROS_NAMESPACE=/ctrl_task_space_ID rosrun panda_torque_mpc pose_publisher.py`  
`roslaunch franka_gazebo panda.launch arm_id:=panda headless:=false use_gripper:=true`  
`roslaunch panda_torque_mpc sim_controllers.launch controller:=ctrl_task_space_ID`  

## Realsense T265 demo with asynchronous MPC
`roslaunch realsense2_camera demo_t265.launch`
`ROS_NAMESPACE=/ctrl_mpc_linearized rosrun panda_torque_mpc pose_publisher.py`
`roslaunch franka_gazebo panda.launch arm_id:=panda headless:=false use_gripper:=true`
`roslaunch panda_torque_mpc sim_controllers.launch controller:=ctrl_mpc_linearized record_joints:=true`
`ROS_NAMESPACE=/ctrl_mpc_linearized rosrun panda_torque_mpc croccodyl_motion_server_node`

# TODOLIST
* Double check if `initialized` topic is streamed when using the real controller (not likely) 
* Switch between the urdf files depending on `load_gripper` argument
* Figure out why measured torque signs are inverted between simulation and real robot.
Do the sign flipping in code automatically
* Switch to `#include <example-robot-data/path.hpp>` + EXAMPLE_ROBOT_DATA_MODEL_DIR
* Refactor log publishers -> LoggingExperiment class with RTpublishers?
* Other logs: update() time, libfranka packet loss stats
