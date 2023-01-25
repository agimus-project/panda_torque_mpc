# panda_torque_mpc
TODO

## Installation
TODO
## Launch
### Simulation
In two different shells:

* `roslaunch franka_gazebo panda.launch`
* `roslaunch panda_torque_controller sim_controllers.launch controller:=<controller-name>`

### Real
TODO

### Available controllers
* `model_pinocchio_vs_franka_controller`: compare Rigid Body Dynamics computation between pinocchio and libfranka
* `pdp_joint_tracking_controller`: Joint impedance control tracking a predefined sinusoidal configuration trajectory 