# panda_torque_mpc
------------------  

# Building the project (tested with Ubuntu 20.05 - ROS noetic)
## conda/mamba env setup
`mamba` is faster but you can use conda interchangeably.

* Create environment:  
`conda create -n panda_control python=3.9`  
* Add package channels:  
`mamba config --env --add channels conda-forge`    
`mamba config --env --add channels robostack-staging`  
* Install ROS and other dependencies:  
`mamba install ros-noetic-desktop-full catkin_tools ros-noetic-combined-robot-hw pinocchio`


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
In two different shells:

* `roslaunch franka_gazebo panda.launch`
* `roslaunch panda_torque_mpc sim_controllers.launch controller:=<controller-name>`

## Real
`robot_ip` and `load_gripper` arguments should be changed accordingly for each launch files

* Bring robot to init position  
`roslaunch franka_example_controllers move_to_start.launch robot_ip:=192.168.102.11 load_gripper:=false robot:=panda`
* Start one of the custom controllers  
`roslaunch panda_torque_mpc real_controllers.launch controller:=<controller-name> robot_ip:=192.168.102.11 load_gripper:=false robot:=panda`

## Custom controllers
The parameters of each controller are defined in `config/controller_configs.yaml`. To run one of them in simulation or real, replace <controller-name> with:
* `model_pinocchio_vs_franka_controller`: compare Rigid Body Dynamics computation between pinocchio and libfranka
* `log_update_dt`: logs ::update time and duration parameters in a csv file to investigate RT control
* `joint_space_ID_controller`: follow joint trajectory reference using different flavors of joint space Inverse Dynamics 
* `task_space_ID_controller`: follow task space end-effector trajectory ($\mathbb{R}^3 or SE(3)) 


## Troubleshooting
* with move_to_start, if you experience a "RealTimeError" kind of message, open  
`rosed franka_control franka_control_node.yaml`  
and change `realtime_config` to `ignore`.  
This means that your computer is not RealTime capable and Franka expects that by default.


# TODOLIST
* Double check if `initialized` topic is streamed when using the real controller (not likely) 
* Switch between the urdf files depending on `load_gripper` argument
* Figure out why measured torque signs are inverted between simulation and real robot.
Do the sign flipping in code automatically
