#!/usr/bin/env python

import itertools
from typing import List, Dict

import rospy

from tf.transformations import quaternion_from_euler

from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Pose
from geometry_msgs.msg import Point, Quaternion


class PosePublisher:
    def __init__(self, name: str) -> None:
        self._node_name = name
        rospy.init_node(self._node_name, anonymous=False)

        if not rospy.has_param("~poses"):
            rospy.logerr(
                f"[{rospy.get_name()}] Param ~poses not fount, but required. Shutting down!"
            )
            rospy.signal_shutdown("Missing ~poses param")

        self._poses_list = rospy.get_param("~poses")
        self._publish_frequency = rospy.get_param("~publish_frequency", 0.25)

        if not self._poses_list:
            rospy.logerr(
                f"[{rospy.get_name()}] No poses to publish passed to the node. Shutting down!"
            )
            rospy.signal_shutdown("Param ~poses not populated")

        self._goal_poses = [
            self._parse_poses(idx, pose) for idx, pose in enumerate(self._poses_list)
        ]

        self._goal_cycle = itertools.cycle(self._goal_poses)

        # -------------------------------
        #   Publishers
        # -------------------------------

        self._pose_pub = rospy.Publisher(
            "absolute_pose_ref", PoseStamped, queue_size=10
        )

        # -------------------------------
        #   Timers
        # -------------------------------

        self._control_loop = rospy.Timer(
            rospy.Duration(1.0 / self._publish_frequency), self._pose_publishing_loop_cb
        )

        rospy.loginfo(
            f"[{rospy.get_name()}] Cycling through {len(self._poses_list)} poses at {self._publish_frequency} Hz."
        )
        rospy.loginfo(f"[{rospy.get_name()}] Node started")

    def _parse_poses(self, idx: int, pose: Dict[str, List[float]]) -> Pose:
        try:
            q = pose["q"] if "q" in pose else quaternion_from_euler(*pose["rpy"])
            return Pose(
                # dark magic association of message objects' parameters
                position=Point(**dict(zip("xyz", pose["xyz"]))),
                orientation=Quaternion(**dict(zip("xyzw", q))),
            )
        except KeyError as e:
            msgs = {"xyz": "position", "q": "rotation", "rpy": "rotation"}
            rospy.logerr(
                f"[{rospy.get_name()}] No {msgs[e.args[0]]} specyfied for pose number {idx}!"
            )

    def _pose_publishing_loop_cb(self, event: rospy.timer.TimerEvent) -> None:
        ps = PoseStamped(
            header=Header(frame_id="world", stamp=rospy.Time.now()),
            pose=next(self._goal_cycle),
        )
        self._pose_pub.publish(ps)
        rospy.loginfo(f"[{rospy.get_name()}] New pose published.")


def main():
    pose_publisher = PosePublisher("pose_publisher")
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
