#!/usr/bin/env python


"""
Sends simulated sinusoidal end effector trajectories.

Topics:
pose_publisher.py --compute_relative_sinusoid -> relative pose reference
pose_publisher.py --compute_absolute_sinusoid -> global pose reference

"""

import yaml
from pathlib import Path
import argparse
import numpy as np
import pinocchio as pin
from example_robot_data import load

import rospy
import rospkg
from geometry_msgs.msg import PoseStamped


parser = argparse.ArgumentParser(
                    prog='pose_publisher',
                    description='publishes sinusoidal pose reference ros msgs for quick tests',
                    epilog='...')
parser.add_argument('-l', '--compute_local_sinusoid', action='store_true', default=False)
parser.add_argument('-a', '--compute_absolute_sinusoid', action='store_true', default=False)
args = parser.parse_args()

if args.compute_local_sinusoid:
    TOPIC_POSE_PUBLISHED = 'motion_capture_pose_ref'

elif args.compute_absolute_sinusoid:
    TOPIC_POSE_PUBLISHED = 'absolute_pose_ref'  

    # use rospack to find start config 
    rospack = rospkg.RosPack()
    start_pose_path = Path(rospack.get_path('panda_torque_mpc')) / 'config' / 'start_joint_pose.yaml'
    with start_pose_path.open() as fp:
        joint_pose_dic = yaml.safe_load(fp)['joint_pose']
    
    r = load('panda')
    # erd model contains fingers by default, does not matter for us
    q = [joint_pose_dic[jname] for jname in sorted(joint_pose_dic.keys())] + [0., 0.]
    T0 = r.framePlacement(np.array(q), r.model.getFrameId('panda_hand'))
else:
    raise ValueError('pass either -l or -a args')

FREQ = 60
DT = 1/FREQ
VERBOSE = True

# DELTA_POSE = np.array([
#     0.0, 0.0, 0.0, 
#     0.0, 0.0, 0.0
# ])

DELTA_POSE = np.array([
    0.1, 0.1, 0.1, 
    -0.3, -0.3, -0.3,
])
 
PERIOD_POSE = np.array([
    4.0, 4.0, 4.0, 
    4.0, 4.0, 4.0, 
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
    dT_ref = compute_sinusoid_pose_delta_reference(delta_pose, period_pose, t)
    return pin.SE3(
        dT_ref.rotation @ T0.rotation,
        T0.translation + dT_ref.translation
    )
    # return  T0 * compute_sinusoid_pose_delta_reference(delta_pose, period_pose, t)


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

            if args.compute_absolute_sinusoid:
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