#!/usr/bin/env python3
import rospy
import numpy as np
import pinocchio as pin
from copy import deepcopy
from threading import Lock
from panda_torque_mpc.initialize_croco_reaching import get_croco_reaching
from panda_torque_mpc.ros_np_multiarray import to_multiarray_f64
from std_msgs.msg import Header, Float64MultiArray
from linear_feedback_controller_msgs.msg import Control, Sensor


class CrocoMotionServer:
    def __init__(self) -> None:
        self.croco_reaching, self.params, self.robot_model = get_croco_reaching()
        self.robot_data = pin.Data(self.robot_model)
        self.rate = rospy.Rate(self.params["publish_rate"])  # 10hz
        self.mutex = Lock()
        self.sensor_msg = Sensor()
        self.control_msg = Control()
        self.x0 = np.zeros(self.robot_model.nq + self.robot_model.nv)
        self.x_guess = np.zeros(self.robot_model.nq + self.robot_model.nv)
        self.u_guess = np.zeros(self.robot_model.nv)
        self.state_subscriber = rospy.Subscriber(
            "robot_sensors",
            Sensor,
            self.sensor_callback,
        )
        self.control_publisher = rospy.Publisher(
            "motion_server_control", Control, queue_size=1
        )
        self.start_time = 0.0
        self.first_solve = False
        self.first_robot_sensor_msg_received = False
        self.first_pose_ref_msg_received = True
        self.iteration = 0
        self.dt = 1.0 / self.params["publish_rate"]

    def sensor_callback(self, sensor_msg):
        with self.mutex:
            self.sensor_msg = deepcopy(sensor_msg)
            if not self.first_robot_sensor_msg_received:
                self.first_robot_sensor_msg_received = True

    def warm_start(self, sensor_msg):
        if self.first_solve:
            self.x_guess[:] = np.concatenate(
                sensor_msg.joint_state.position,
                np.zeros(len(sensor_msg.joint_state.velocity)),
            )
            self.u_guess[:] = pin.computeGeneralizedGravity(
                self.robot_model, self.robot_data, sensor_msg.joint_state.position
            )
            xs = [np.array(x) for x in self.x_guess]
            us = [np.array(u) for u in self.u_guess]
            nb_iteration = 500
            self.first_solve = False
        else:
            xs = [np.array(x) for x in self.croco_reaching.solver.xs]
            us = [np.array(x) for x in self.croco_reaching.solver.us]
            nb_iteration = 500  # 1

        return xs, us, nb_iteration

    def solve_and_send(self):
        with self.mutex:
            sensor_msg = deepcopy(self.sensor_msg)

        wait_for_input = (
            not self.first_robot_sensor_msg_received
            or not self.first_pose_ref_msg_received
        )
        if wait_for_input:
            rospy.loginfo_throttle(3, "Waiting until we receive a sensor message.")
            with self.mutex:
                self.start_time = sensor_msg.header.stamp.to_sec()
            return

        x0 = np.concatenate(
            [sensor_msg.joint_state.position, sensor_msg.joint_state.velocity]
        )
        self.croco_reaching.solver.problem.x0 = x0
        self.croco_reaching.set_ee_ref_placement(
            sensor_msg.header.stamp.to_sec() - self.start_time, True, 1.0
        )
        self.croco_reaching.set_posture_ref(x0)

        x_guess, u_guess, nb_iteration_max = self.warm_start(sensor_msg)
        self.croco_reaching.solve(x_guess, u_guess, nb_iteration_max)

        self.control_msg.header = Header()
        self.control_msg.header.stamp = rospy.Time.now()
        self.control_msg.feedback_gain = to_multiarray_f64(
            self.croco_reaching.ricatti_mat
        )
        self.control_msg.feedforward = to_multiarray_f64(self.croco_reaching.tau_ff)
        self.control_msg.initial_state = sensor_msg
        self.control_publisher.publish(self.control_msg)

    def run(self):
        while not rospy.is_shutdown():
            self.solve_and_send()
            self.rate.sleep()


def crocco_motion_server_node():
    rospy.init_node("croccodyl_motion_server_node_py", anonymous=True)
    node = CrocoMotionServer()
    node.run()


if __name__ == "__main__":
    try:
        crocco_motion_server_node()
    except rospy.ROSInterruptException:
        pass
