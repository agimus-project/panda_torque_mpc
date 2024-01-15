#!/usr/bin/env python


"""
Sends simulated sinusoidal end effector trajectories.

Topics:
pose_publisher.py --compute_relative_sinusoid -> relative pose reference
pose_publisher.py --compute_absolute_sinusoid -> global pose reference

"""

import numpy as np
import pinocchio as pin
import rospy
import argparse

from geometry_msgs.msg import PoseStamped


parser = argparse.ArgumentParser(
                    prog='pose_publisher',
                    description='Republishes transformation from tf as pose goal',
                    epilog='...')
parser.add_argument('-l', '--compute_local_sinusoid', action='store_true', default=False)  # on/off flag
parser.add_argument('-g', '--compute_global_sinusoid', action='store_true', default=False)  # on/off flag
args = parser.parse_args()

if args.compute_local_sinusoid:
    TOPIC_POSE_PUBLISHED = 'motion_capture_pose_ref'
elif args.compute_local_sinusoid:
    TOPIC_POSE_PUBLISHED = 'absolute_pose_ref'  # TODO
    T0  = ...  # TODO
else:
    raise ValueError('pass either -l or -g args')

FREQ = 60
DT = 1/FREQ
VERBOSE = True

# DELTA_POSE = np.array([
#     0.0, 0.0, 0.0, 
#     0.0, 0.0, 0.0
# ])

DELTA_POSE = np.array([
    0.0, 0.0, 0.1, 
    0.2, 0.2, 0.2
])
 
PERIOD_POSE = np.array([
    4.0, 4.0, 4.0, 
    5.0, 5.0, 5.0
])


def compute_sinusoid_pose_delta_reference(delta_pose, period_pose, t):

    """
    Compute a delta SE(3) pose reference sinusoid pose in
    The subscriber needs to compose this local pose with a reference initial pose.

    Ai and Ci obtained for each joint using constraints:
    T(t=0.0) = 0
    T(t=period/2) = Exp(delta_pose)
    """

    w = 2*np.pi/period_pose
    a = - delta_pose
    c = delta_pose
    
    nu = a * np.cos(w * t) + c

    # Adopt a R3xSO(3) representation to decouple translation and orientation
    dp_ref = nu[:3]
    dR_ref = pin.exp3(nu[3:])

    return pin.SE3(dR_ref, dp_ref)


def compute_sinusoid_pose_reference(delta_pose, period_pose, T0: pin.SE3, t):
    return T0 * compute_sinusoid_pose_delta_reference(delta_pose, period_pose, t)


if __name__ == '__main__':
    try:
        pub = rospy.Publisher(TOPIC_POSE_PUBLISHED, PoseStamped, queue_size=10)
    
        rospy.init_node('pose_publisher', anonymous=False)
        rate = rospy.Rate(FREQ)

        t0 = rospy.Time.now()
        while not rospy.is_shutdown():
            t = rospy.Time.now()

            print('t-t0', (t-t0).to_sec())

            
            if args.compute_local_sinusoid:
                T_ref = compute_sinusoid_pose_delta_reference(DELTA_POSE, PERIOD_POSE, (t - t0).to_sec())

            if args.compute_global_sinusoid:
                T_ref = compute_sinusoid_pose_reference(DELTA_POSE, PERIOD_POSE, T0, (t - t0).to_sec())

            msg = PoseStamped()
            msg.header.stamp.secs = t.secs
            msg.header.stamp.nsecs = t.nsecs

            msg.pose.position.x = T_ref.translation[0]
            msg.pose.position.y = T_ref.translation[1]
            msg.pose.position.z = T_ref.translation[2]
            q = pin.Quaternion(T_ref.rotation)
            msg.pose.orientation.x = q.x
            msg.pose.orientation.y = q.y
            msg.pose.orientation.z = q.z
            msg.pose.orientation.w = q.w

            pub.publish(msg)

            rate.sleep()

    except rospy.ROSInterruptException as e:
        print(e)