# panda_torque_mpc
------------------
Various torque controllers, building toward torque MPC of Panda manipulator. 

# Building
## conda/mamba env setup
`mamba` is faster but you can use conda interchangeably.

* Create environment:  
`conda create -n panda_control python=3.9`
`conda activate panda_control`

* Install build tools:  
`mamba install compilers cmake pkg-config make ninja -c conda-forge`

* Install ROS related packages:  
`mamba install ros-noetic-desktop ros-noetic-combined-robot-hw catkin_tools ros-noetic-realsense2-camera -c conda-forge -c robostack-staging`

* Install robotics control libraries:  
`mamba install pinocchio tsid example-robot-data crocoddyl -c conda-forge`

## Franka panda
TLDR;  
`mamba install ros-noetic-libfranka ros-noetic-franka-ros -c robostack-staging -c conda-forge`

### libfranka
Check [compatibility](https://frankaemika.github.io/docs/compatibility.html "FCI-libfranka compatibily matrix") of your Franka Control Interface version to determine which libfranka version to install.

At CIIRC, FCI version == 4.2.2  --> libfranka version >= 0.9.1 < 0.10.0. `robostack` conda channel provides 9.2 version which requires python 3.9 to be installed (hence `python=3.9` when creating the conda env).   
`mamba install ros-noetic-libfranka -c robostack-staging -c conda-forge`

## Franka ROS
### From conda
`mamba install ros-noetic-franka-ros -c robostack-staging -c conda-forge`

### From source
In your catkin workspace src directory  
`git clone git@github.com:frankaemika/franka_ros.git`  
`catkin build franka_ros -DCMAKE_BUILD_TYPE=RELEASE`  

## Other packages to clone
`git clone git@github.com:loco-3d/linear-feedback-controller-msgs.git`

## Crocoddyl installation from sources
NOTE: after benchmarking, crocoddyl from conda-forge is just as good as single from source.
Compiling with multithreading makes things worse.

Step 1: 
Compile crocoddyl in active panda_control conda env but overidding conda compiler with system one
```bash
CROCO_INSTALL=<your/own/path>
cd <crocoddyl-repo-dir>/build
# choose your system compiler (linker error when using the conda g++ bin!!). E.G.:
CXX=/usr/bin/clang++
cmake .. -DBUILD_EXAMPLES=OFF -DBUILD_PYTHON_INTERFACE=OFF -DCMAKE_INSTALL_PREFIX=$CROCO_INSTALL
# or with multithreading ON
cmake .. -DBUILD_EXAMPLES=OFF -DBUILD_PYTHON_INTERFACE=OFF -DCMAKE_INSTALL_PREFIX=$CROCO_INSTALL -BUILD_WITH_MULTITHREADS=ON
make -j4
make install
```

Step 2:
Make sure that the CXX env variable is set to the conda compiler (if not: `conda deactivate; conda activate panda_control`). 
Also make sure crocoddyl is not in your environment otherwise it will be used. 
Then:
`CMAKE_BUILD_PARALLEL_LEVEL=4 catkin build panda_torque_mpc -Dcrocoddyl_DIR=$CROCO_INSTALL/lib/cmake/crocoddyl/ -DCMAKE_BUILD_TYPE=RELEASE`

## 
# Launch
## Simulation
In two different shells (change use_gripper according to which urdf model you use for control):  

* `roslaunch franka_gazebo panda.launch arm_id:=panda headless:=false use_gripper:=true`
* `roslaunch panda_torque_mpc sim_controllers.launch controller:=<controller-name>`

## Real
`robot_ip` and `load_gripper` arguments should be changed accordingly for each launch files

* Bring robot to init position  
`roslaunch panda_torque_mpc move_to_start.launch robot_ip:=192.168.102.11 load_gripper:=true robot:=panda`
* Start one of the custom controllers  
`roslaunch panda_torque_mpc real_controllers.launch controller:=<controller-name> robot_ip:=192.168.102.11 load_gripper:=true robot:=panda`

## Custom controllers
The parameters of each controller are defined in `config/controller_configs.yaml`. To run one of them in simulation or real, replace <controller-name> with:
* `ctrl_model_pinocchio_vs_franka`: compare Rigid Body Dynamics computation between pinocchio and libfranka
* `ctrl_log_update_dt`: logs ::update time and duration parameters in a csv file to investigate RT control
* `ctrl_playback_pd_plus`: reads a joint trajectory stored in csv files q.csv, v.csv, tau.csv and plays it back using PD+ 
* `ctrl_joint_space_ID`: follow joint trajectory reference using different flavors of joint space Inverse Dynamics 
* `ctrl_task_space_ID`: follow task space end-effector trajectory ($\mathbb{R}^3$ or SE(3)) 
* `ctrl_mpc_croco`: synchronously solving of OCP using crocoddyl and sending the first torque command -> limited to very short horizons to avoid breaking real time constraint 
* `ctrl_mpc_linearized`: asynchronous execution a linearized control reference from OCP solver running in another node (croccodyl_motion_server_node) using Ricatti gains -> very few computation, no update() skipped

## Realsense T265 demo (launch in this order in different shells)
```bash
roslaunch realsense2_camera demo_t265.launch  
ROS_NAMESPACE=/ctrl_task_space_ID rosrun panda_torque_mpc pose_publisher.py  
roslaunch franka_gazebo panda.launch arm_id:=panda headless:=false use_gripper:=true  
roslaunch panda_torque_mpc sim_controllers.launch controller:=ctrl_task_space_ID  
```

## Realsense T265 demo with asynchronous MPC (simu)
```bash
roslaunch realsense2_camera demo_t265.launch
roslaunch franka_gazebo panda.launch arm_id:=panda headless:=false use_gripper:=true
roslaunch panda_torque_mpc sim_controllers.launch controller:=ctrl_mpc_linearized record_joints:=tru
ROS_NAMESPACE=/ctrl_mpc_linearized rosrun panda_torque_mpc croccodyl_motion_server_node
ROS_NAMESPACE=/ctrl_mpc_linearized rosrun panda_torque_mpc pose_publisher.py
```

## Realsense T265 demo with asynchronous MPC (real)
```bash
roslaunch realsense2_camera demo_t265.launch
roslaunch panda_torque_mpc real_controllers.launch controller:=ctrl_mpc_linearized robot_ip:=192.168.102.11 load_gripper:=true robot:=panda
ROS_NAMESPACE=/ctrl_mpc_linearized rosrun panda_torque_mpc croccodyl_motion_server_node
ROS_NAMESPACE=/ctrl_mpc_linearized rosrun panda_torque_mpc pose_publisher.py
```

# TODOLIST
* Double check if `initialized` topic is streamed when using the real controller (not likely) 
* Switch between the urdf files depending on `load_gripper` argument
* Figure out why measured torque signs are inverted between simulation and real robot.
Do the sign flipping in code automatically
* Switch to `#include <example-robot-data/path.hpp>` + EXAMPLE_ROBOT_DATA_MODEL_DIR
* Refactor log publishers -> LoggingExperiment class with RTpublishers?
* Other logs: update() time, libfranka packet loss stats
