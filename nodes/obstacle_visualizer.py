#!/usr/bin/env python

from typing import List, Dict

import rospy

from std_msgs.msg import Header
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point, Quaternion, Vector3
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, Header

from SDFGenerator import SDFGenerator


class ObstaclesVisualizer:
    def __init__(self, name: str) -> None:
        self._node_name = name
        rospy.init_node(self._node_name, anonymous=False)

        obstacle_idx = 1

        if not rospy.has_param("~obstacle1"):
            rospy.logerr(
                f"[{rospy.get_name()}] Param obstacle1 not found. "
                "No collision avoidance with an external obstacle computed!"
            )           

        self._obstacles_infos = {}
        while rospy.has_param("~obstacle" + str(obstacle_idx)):
            obstacle_name = "~obstacle" + str(obstacle_idx)
            self._obstacles_infos.update(
                {obstacle_name: rospy.get_param(obstacle_name)}
            )
            obstacle_idx += 1

        sdf_generator = SDFGenerator()
        self._spawn_model_requests = []
        self._markers = MarkerArray()
        header = Header(frame_id="world", stamp=rospy.Time.now())
        for iter, (key, obstacle) in enumerate(self._obstacles_infos.items()):
            pose=self._parse_poses(obstacle)
            obstacle_name = key.lstrip("~")
            sp_req = SpawnModelRequest(
                model_name = obstacle_name,
                robot_namespace = "",
                initial_pose = pose,
                reference_frame = header.frame_id
            )
            m = Marker(
                header=header,
                ns="",
                id=iter,
                action=Marker.MODIFY,
                pose = pose,
                color=ColorRGBA(r=0.58, g=0.0, b=0.98, a=0.7),
                lifetime=rospy.Duration(0.1),
                frame_locked=False,
            )
            if obstacle["type"] == "sphere":
                m.scale = Vector3(**dict(zip("xyz", [obstacle["radius"] * 2] * 3)))
                m.type = Marker.SPHERE
                sp_req.model_xml = sdf_generator.generate_sphere(key[1:], obstacle["radius"])
            if obstacle["type"] == "cylinder":
                m.scale = Vector3(x = obstacle["radius"], y = obstacle["radius"], z = obstacle["halfLength"])
                m.type = Marker.CYLINDER
                sp_req.model_xml = sdf_generator.generate_cylinder(key[1:], obstacle["radius"],obstacle["halfLength"] )
            if obstacle["type"] == "box":
                m.scale = Vector3(x = obstacle["x"], y = obstacle["y"], z = obstacle["z"])
                m.type = Marker.CUBE
                sp_req.model_xml = sdf_generator.generate_box(key[1:], [obstacle["x"],obstacle["y"], obstacle["z"] ])
            if obstacle["type"] == "capsule":
                m.scale = Vector3(x = obstacle["radius"], y = obstacle["radius"], z = obstacle["halfLength"])
                m.type = Marker.CYLINDER
                sp_req.model_xml = sdf_generator.generate_capsule(key[1:], obstacle["radius"],obstacle["halfLength"] )              
            self._spawn_model_requests.append(sp_req)
            self._markers.markers.append(m)


        if rospy.get_param("~spawn_in_gz", default=False):   
            rospy.wait_for_service("/gazebo/spawn_urdf_model")
            try:
                model_spawner = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
                for sp_req in self._spawn_model_requests:
                    resp = model_spawner(sp_req)
                    if not resp.success:
                        rospy.logerr("spawning model didn't work.")
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)

        # -------------------------------
        #   Publishers
        # -------------------------------
        self._marker_pub = rospy.Publisher(
            "obstacle_markers", MarkerArray, queue_size=10
        )

        # -------------------------------
        #   Timers
        # -------------------------------

        # Publish markers at 10 Hz        
        self._control_loop = rospy.Timer(
            rospy.Duration(0.1), self._publish_markers_cb,
        )
        rospy.loginfo(f"[{rospy.get_name()}] Node started")

    def _parse_poses(self, obstacle: Dict[str, List[float]]) -> Pose:
        return Pose(
            # dark magic association of message objects' parameters
            position=Point(**dict(zip("xyz", obstacle["translation"]))),
            orientation=Quaternion(**dict(zip("xyzw", obstacle["rotation"]))),
        )

    def _publish_markers_cb(self, kwargs) -> None:
        self._marker_pub.publish(self._markers)
      


def main() -> None:

    marker_publisher = ObstaclesVisualizer("obstacles_visualizer")
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass