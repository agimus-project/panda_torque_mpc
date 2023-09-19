#!/usr/bin/env python

"""
Sources of info for tf2
https://w3.cs.jmu.edu/spragunr/CS354_S19/lectures/tf/tf2_demo.py
https://cw.fel.cvut.cz/wiki/_media/courses/aro/tutorials/tf_slides.pdf
"""

import numpy as np
import pinocchio as pin
import rospy
import argparse

from geometry_msgs.msg import PoseStamped


parser = argparse.ArgumentParser(
                    prog='pose_publisher_simulated_object_mvt',
                    description='Publishes camera pose reference as if a virtual object was movint in the scene',
                    epilog='...')
parser.add_argument('-c', '--compute_sinusoid', action='store_true')  # on/off flag
args = parser.parse_args()

TOPIC_POSE_PUBLISHED = 'pose_object_rel'

FREQ = 60
DT = 1/FREQ
VERBOSE = True

T_o0_o_ref = pin.SE3.Identity()


CHANGE_AFTER_SECS = 3.0

# delta in object reference frame
# For cheezit, looking straight at the front: X-Y-Z = BACK-RIGHT-UP
DELTA_T = np.array([0.0,0.0,0.0])
# MIN TESTED: -10.0,-10.0,-40.0
# MAX TESTED: +10.0,20.0,+40.0
DELTA_W_DEG = np.array([0.0,0.0,20.0])
R_o0_o = pin.exp3(np.deg2rad(DELTA_W_DEG))

def talker():
    pub = rospy.Publisher(TOPIC_POSE_PUBLISHED, PoseStamped, queue_size=10)
    
    rospy.init_node('pose_publisher', anonymous=False)
    rate = rospy.Rate(FREQ)

    change_done = False
    t0 = rospy.Time.now()
    while not rospy.is_shutdown():
        t = rospy.Time.now()

        print('t-t0', (t-t0).to_sec())

        if not change_done and (t-t0).to_sec() > CHANGE_AFTER_SECS:
            T_o0_o_ref.translation += DELTA_T
            T_o0_o_ref.rotation = T_o0_o_ref.rotation @  R_o0_o
            change_done = True
            print('\n\n\n\n\n\nGO!\n\n\n\n\n\n')

        msg = PoseStamped()
        msg.header.stamp.secs = t.secs
        msg.header.stamp.nsecs = t.nsecs
        msg.header.frame_id = "panda_link0"

        msg.pose.position.x = T_o0_o_ref.translation[0]
        msg.pose.position.y = T_o0_o_ref.translation[1]
        msg.pose.position.z = T_o0_o_ref.translation[2]
        q = pin.Quaternion(T_o0_o_ref.rotation)
        msg.pose.orientation.x = q.x
        msg.pose.orientation.y = q.y
        msg.pose.orientation.z = q.z
        msg.pose.orientation.w = q.w

        pub.publish(msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException as e:
        print(e)