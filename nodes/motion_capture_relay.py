#!/usr/bin/python3

import math

import rospy
import tf2_ros

from geometry_msgs.msg import Point, Pose, PoseStamped, Vector3, Quaternion
from std_msgs.msg import ColorRGBA, Duration, Header
from visualization_msgs.msg import Marker


class MoCapRelay:
    def __init__(self, name: str) -> None:
        self._node_name = name
        rospy.init_node(self._node_name, anonymous=False)

        if not rospy.has_param("~sphere_radious"):
            rospy.logerr(
                f"[{rospy.get_name()}] Param ~sphere_radious not found, but required. Shutting down!"
            )
            rospy.signal_shutdown("Missing ~sphere_radious param")
            return

        if not rospy.has_param("~relay"):
            rospy.loginfo(
                f"[{rospy.get_name()}] No parameter ~relay. Running as generator!"
            )

        self._is_relay = rospy.get_param("~relay", False)
        if self._is_relay:
            rospy.loginfo(
                f"[{rospy.get_name()}] Node configured to run as offseted relay!"
            )
        else:
            rospy.loginfo(
                f"[{rospy.get_name()}] Node configured to run as motion generator!"
            )

        self._sphere_radious = rospy.get_param("~sphere_radious")
        self._target_frame_id = rospy.get_param("~frame_id", "world")
        self._offset_x = rospy.get_param(f"~motion_generator/offset/x", 0.0)
        self._offset_y = rospy.get_param(f"~motion_generator/offset/y", 0.0)
        self._offset_z = rospy.get_param(f"~motion_generator/offset/z", 1.0)

        self._motion_axis = rospy.get_param("~motion_generator/motion/axis", "x")
        self._motion_mag = rospy.get_param("~motion_generator/motion/mag", 0.1)
        self._motion_freq = rospy.get_param("~motion_generator/motion/frequency", 1.0)
        self._publish_frequency = rospy.get_param("~motion_generator/publish_frequency", 120.0)

        # -------------------------------
        #   TF2 subscribers
        # -------------------------------

        self._tf_buffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._tf_buffer)

        # -------------------------------
        #   Publishers
        # -------------------------------

        self._pose_pub = rospy.Publisher("obstacle_pose", PoseStamped, queue_size=10)
        self._marker_pub = rospy.Publisher("obstacle_marker", Marker, queue_size=10)

        # -------------------------------
        #   Subscribers
        # -------------------------------

        if self._is_relay:
            self._mocap_sub = rospy.Subscriber(
                "mocap_pose", PoseStamped, self._mocap_subscription_cb, queue_size=10
            )

        # -------------------------------
        #   Timers
        # -------------------------------

        if not self._is_relay:
            self._control_loop = rospy.Timer(
                rospy.Duration(1.0 / self._publish_frequency), self._sine_pub_cb
            )

        rospy.loginfo(f"[{rospy.get_name()}] Node started")

    def _sine_pub_cb(self, event: rospy.timer.TimerEvent) -> None:
        ps = PoseStamped(
            header=Header(frame_id=self._target_frame_id, stamp=rospy.Time.now()),
            pose=Pose(
                position=Point(x=self._offset_x, y=self._offset_y, z=self._offset_z),
                orientation=Quaternion(),
            ),
        )
        disp = getattr(ps.pose.position, self._motion_axis)
        disp += self._motion_mag * math.sin(
            rospy.Time().now().to_sec() * self._motion_freq * 2.0
        )
        setattr(ps.pose.position, self._motion_axis, disp)
        self._publish_pose(ps)

    def _mocap_subscription_cb(self, data: PoseStamped) -> None:
        try:
            converted_pose = self._tf_buffer.transform(
                data,
                self._target_frame_id,
                rospy.Duration(0.1)
            )
            self._publish_pose(converted_pose)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"[{rospy.get_name()}] Failed to transform from frame"
            f" {data.header.frame_id} to frame {self._target_frame_id}. Reason: {str(e)}")

    def _publish_pose(self, pose: PoseStamped) -> None:
        m = Marker(
            header=pose.header,
            ns="",
            id=0,
            type=Marker.SPHERE,
            action=Marker.MODIFY,
            pose=pose.pose,
            # Sphere of equal radious
            scale=Vector3(**dict(zip("xyz", [self._sphere_radious] * 3))),
            color=ColorRGBA(r=0.58, g=0.0, b=0.98, a=0.7),
            lifetime=rospy.Duration(0.1),
            frame_locked=False,
        )
        self._pose_pub.publish(pose)
        self._marker_pub.publish(m)


def main():
    motion_capture_relay = MoCapRelay("motion_capture_relay")
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
