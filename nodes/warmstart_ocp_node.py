#!/usr/bin/env python3

## Class heavily inspired by the work of Sebastien Kleff : https://github.com/machines-in-motion/minimal_examples_crocoddyl
import sys
from os.path import *

import numpy as np
import crocoddyl
import pinocchio as pin
import mim_solvers
from typing import Any

import rospy

import time

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from linear_feedback_controller_msgs.msg import Sensor
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class OCPPandaReaching:
    """This class is creating a optimal control problem of a panda robot reaching for a target while taking auto collisions into consideration"""

    def __init__(
        self,
        rmodel: pin.Model,
        TARGET_POSE: pin.SE3,
        T: int,
        dt: float,
        x0: np.ndarray,
        WEIGHT_xREG=1e-2,
        WEIGHT_xREG_TERM = 1e-2,
        WEIGHT_uREG = 1e-4,
        WEIGHT_GRIPPER_POSE=10,
        WEIGHT_GRIPPER_POSE_TERM = 10,
        WEIGHT_LIMIT = 1e-1,
    ) -> None:
        """Creating the class for optimal control problem of a panda robot reaching for a target while taking auto collision into consideration

        Args:
            rmodel (pin.Model): pinocchio Model of the robot
            TARGET_POSE (pin.SE3): Pose of the target in WOLRD ref
            T (int): Number of nodes in the trajectory
            dt (float): _description_
            x0 (np.ndarray): _description_
            WEIGHT_xREG (_type_, optional): _description_. Defaults to 1e-1.
            WEIGHT_GRIPPER_POSE (int, optional): _description_. Defaults to 10.
        """

        self._rmodel = rmodel
        self._TARGET_POSE = TARGET_POSE
        self._T = T
        self._dt = dt
        self._x0 = x0

        # Weights
        self._WEIGHT_xREG = WEIGHT_xREG
        self._WEIGHT_xREG_TERM = WEIGHT_xREG_TERM

        self._WEIGHT_uREG = WEIGHT_uREG

        self._WEIGHT_GRIPPER_POSE = WEIGHT_GRIPPER_POSE
        self._WEIGHT_GRIPPER_POSE_TERM = WEIGHT_GRIPPER_POSE_TERM
        self._WEIGHT_LIMIT = WEIGHT_LIMIT
        # Data models
        self._rdata = rmodel.createData()

    def __call__(self) -> Any:
        "Setting up croccodyl OCP"

        # Stat and actuation model
        self._state = crocoddyl.StateMultibody(self._rmodel)
        self._actuation = crocoddyl.ActuationModelFull(self._state)

        # Running & terminal cost models
        self._runningCostModel = crocoddyl.CostModelSum(self._state)
        self._terminalCostModel = crocoddyl.CostModelSum(self._state)

        ### Creation of cost terms

        # State Regularization cost
        xResidual = crocoddyl.ResidualModelState(self._state, self._x0)
        xRegCost = crocoddyl.CostModelResidual(self._state, xResidual)

        # Control Regularization cost 
        uResidual = crocoddyl.ResidualModelControl(self._state)
        uRegCost = crocoddyl.CostModelResidual(self._state, uResidual)
        

        # End effector frame cost
        framePlacementResidual = crocoddyl.ResidualModelFrameTranslation(
            self._state,
            self._rmodel.getFrameId("panda_hand"),
            self._TARGET_POSE.translation,
        )
        goalTrackingCost = crocoddyl.CostModelResidual(
            self._state, framePlacementResidual
        )

        # Adding costs to the models
        self._runningCostModel.addCost("stateReg", xRegCost, self._WEIGHT_xREG)
        self._runningCostModel.addCost("ctrlRegGrav", uRegCost, self._WEIGHT_uREG)
        self._runningCostModel.addCost("gripperPoseRM", goalTrackingCost, self._WEIGHT_GRIPPER_POSE)    
        self._terminalCostModel.addCost("stateReg", xRegCost, self._WEIGHT_xREG)
        self._terminalCostModel.addCost(
            "gripperPose", goalTrackingCost, self._WEIGHT_GRIPPER_POSE
        )


        # Create Differential Action Model (DAM), i.e. continuous dynamics and cost functions
        self._running_DAM = crocoddyl.DifferentialActionModelFreeFwdDynamics(
            self._state, self._actuation, self._runningCostModel
        )
        self._terminal_DAM = crocoddyl.DifferentialActionModelFreeFwdDynamics(
            self._state, self._actuation, self._terminalCostModel
        )

        # Create Integrated Action Model (IAM), i.e. Euler integration of continuous dynamics and cost
        self._runningModel = crocoddyl.IntegratedActionModelEuler(
            self._running_DAM, self._dt
        )
        self._terminalModel = crocoddyl.IntegratedActionModelEuler(
            self._terminal_DAM, 0.0
        )

        self._runningModel.differential.armature = 1 *  np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.])
        self._terminalModel.differential.armature =  1 * np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.])


        problem = crocoddyl.ShootingProblem(
            self._x0, [self._runningModel] * self._T, self._terminalModel
        )
        # Create solver + callbacks

        # Define solver
        ddp = mim_solvers.SolverSQP(problem)
        ddp.use_filter_line_search = False
        ddp.termination_tolerance = 1e-3
        ddp.max_qp_iters = 1000
        ddp.with_callbacks = True 

        return ddp

class WarmstartOCPNode:
    def __init__(self, name: str) -> None:
        self._name = name
        rospy.init_node(self._name, anonymous=False)

        # Building the model
        urdf_path = join(dirname(dirname(str(abspath(__file__)))), "urdf/robot.urdf")
        rmodel_full = pin.buildModelFromUrdf(urdf_path)

        q0 = pin.neutral(rmodel_full)

        self._rmodel = pin.buildReducedModel(rmodel_full, [8,9], q0)
        self._T = 10 # Shooting nodes
        self._dt = 0.05 # dt_ocp

        self._state = None
        self._last_pose_ref = None

        # Subscribers
        self._goal_sub = rospy.Subscriber('absolute_pose_ref', PoseStamped, self._goal_cb, queue_size=5)
        self._state_sub = rospy.Subscriber('robot_sensors', Sensor, self._state_cb, queue_size=1)

        # Publishers
        self._warm_pub = rospy.Publisher("ocp_warmstart", JointTrajectory, queue_size=5)

        while not self._state and not rospy.is_shutdown():
            rospy.loginfo(f"[{self._name}] Waiting for state message to arrive...")
            rospy.spin()
            time.sleep(0.5)

        rospy.loginfo(f"[{self._name}] OCP warmstart node started!")

    def _goal_cb(self, pose_ref: PoseStamped) -> None:
        p = np.array([
            pose_ref.pose.position.x,
            pose_ref.pose.position.y,
            pose_ref.pose.position.z
        ])

        if self._last_pose_ref is None or np.linalg.norm(self._last_pose_ref - p) > 1e-15:
            x0 = np.concatenate([self._state.position, self._state.velocity])

            rospy.loginfo(f"[{self._name}] New goal at: {p}")
            rospy.loginfo(f"[{self._name}] Solving for warmstart...")
            tmp = pin.SE3.Identity()
            tmp.translation = p
            ddp = OCPPandaReaching(
                self._rmodel,
                # pin.SE3(np.eye(3), np.array(pose_ref.pose.position)),
                tmp,
                self._T, 
                self._dt,
                x0
            )()

            xs_init = [x0 for i in range(self._T+1)]
            us_init = ddp.problem.quasiStatic(xs_init[:-1])
            # Solve
            ddp.solve(xs_init, us_init, maxiter=100)

            rospy.loginfo(f"[{self._name}] Warmstart solution found.")

            def get_jtp(x: np.array, u: np.array) -> JointTrajectoryPoint:
                jtp = JointTrajectoryPoint()
                jtp.positions = x[0:x.shape[0]//2]
                jtp.velocities = x[x.shape[0]//2:-1]
                jtp.effort = u
                return jtp

            traj = JointTrajectory()
            traj.header.stamp = rospy.Time.now()
            traj.points = [
                get_jtp(x, u) for x, u in zip(ddp.xs.tolist(), ddp.us.tolist())
            ]

            self._warm_pub.publish(traj)

        self._last_pose_ref = p


    def _state_cb(self, msg: Sensor) -> None:
        self._state = msg.joint_state

def main() -> None:
    warmstart_ocp_node = WarmstartOCPNode('warmstart_ocp_node')
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except  rospy.ROSInterruptException:
        pass