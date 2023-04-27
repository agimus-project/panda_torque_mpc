#!/usr/bin/env python

"""
Sources of info for tf2
https://w3.cs.jmu.edu/spragunr/CS354_S19/lectures/tf/tf2_demo.py
https://cw.fel.cvut.cz/wiki/_media/courses/aro/tutorials/tf_slides.pdf
"""


import numpy as np
import pinocchio as pin
import rospy
import tf2_ros
import argparse

from panda_torque_mpc.msg import PoseTaskGoal


parser = argparse.ArgumentParser(
                    prog='pose_publisher',
                    description='Republishes transformation from tf as pose goal',
                    epilog='...')
parser.add_argument('-s', '--visual_servoing', action='store_true')  # on/off flag
parser.add_argument('-c', '--compute_sinusoid', action='store_true')  # on/off flag
args = parser.parse_args()


LISTEN_TO_TF = not args.compute_sinusoid
VISUAL_SERVOING = args.visual_servoing
if VISUAL_SERVOING:
    TOPIC_POSE_PUBLISHED = 'ee_pose_ref_visual_servoing'
    # DELAY_AVOID_EXTRAP = 0.15  # ICG
    DELAY_AVOID_EXTRAP = 0.2  # Apriltag
else:
    TOPIC_POSE_PUBLISHED = 'ee_pose_ref'
    DELAY_AVOID_EXTRAP = 0.05

print('LISTEN_TO_TF: ', LISTEN_TO_TF)
print('VISUAL_SERVOING: ', VISUAL_SERVOING)
print('Publish pose goal on: ', TOPIC_POSE_PUBLISHED)

FREQ = 60
DT = 1/FREQ
VERBOSE = True
# we want T_wc
camera_pose_frame = "camera_pose_frame"  # moving "camera=c" frame
world_frame = "camera_odom_frame"  # static inertial "world=w" frame

# We want T_c_o
camera_color_optical_frame = "camera_color_optical_frame"
object_frame = "object_frame"



# Documentation is not clear about which transformation is retrieved by lookup_transform (target/source or source/target)
# After testing with T265 node, it seems it is T_wc, s.t. 
# w_vec = T_wc * c_vec
if VISUAL_SERVOING:
    base_frame, target_frame = camera_color_optical_frame, object_frame
else:  # T265
    base_frame, target_frame = world_frame, camera_pose_frame



def compute_sinusoid_pose_delta_reference(delta_nu, period_nu, t):

    """
    Compute a local SE(3) pose sinusoid pose in

    Ai and Ci obtained for each joint using constraints:
    T(t=0.0) = 0
    T(t=period/2) = Exp(delta_nu)

    The subscriber needs to compose this local pose with a reference initial pose.
    """

    print('compute_sinusoid_pose_delta_reference:')
    print(t)

    w = 2*np.pi/period_nu
    a = - delta_nu
    c = delta_nu
    
    nu =            a * np.cos(w * t) + c
    nu_bt_ref =     -w * a * np.sin(w*t)
    dnu_bt_ref = -w**2 * a * np.cos(w*t)

    T_bt_ref = pin.exp(nu)

    return T_bt_ref, nu_bt_ref, dnu_bt_ref




# DELTA_NU = np.array([
#     0.0, 0.0, 0.0, 
#     0.0, 0.0, 0.0
# ])

# DELTA_NU = np.array([
#     0.04, 0.04, 0.04, 
#     0.0, 0.0, 0.0
# ])

DELTA_NU = np.array([
    0.07, 0.08, 0.09, 
    0.0, 0.0, 0.0
])

 
PERIOD_NU = np.array([
    4.0, 4.0, 4.0, 
    5.0, 5.0, 5.0
])


def talker():
    pub = rospy.Publisher(TOPIC_POSE_PUBLISHED, PoseTaskGoal, queue_size=10)
    
    rospy.init_node('pose_publisher', anonymous=False)
    rate = rospy.Rate(FREQ)

    if LISTEN_TO_TF:
        # receives tf2 messages from the /tf topic and buffers them for 10 second (by default)
        tfBuffer = tf2_ros.Buffer()

        tf2_ros.TransformListener(tfBuffer)

        dt_to_fill_the_buffer = 1.0
        print(f'Filling tf transform buffer for {dt_to_fill_the_buffer} seconds')
        rospy.sleep(dt_to_fill_the_buffer)

    t0 = rospy.Time.now()
    while not rospy.is_shutdown():
        t = rospy.Time.now()

        print('t-t0', (t-t0).to_sec())

        if LISTEN_TO_TF:
            try:
                # Add delay to make sure we are not extrapolating the tf lookup
                # T_base_target
                T_bt = tfBuffer.lookup_transform(base_frame, target_frame, t - rospy.Duration(DELAY_AVOID_EXTRAP))

                T_bt_ref = pin.XYZQUATToSE3(
                    [
                        T_bt.transform.translation.x, T_bt.transform.translation.y, T_bt.transform.translation.z,
                        T_bt.transform.rotation.x, T_bt.transform.rotation.y, T_bt.transform.rotation.z, T_bt.transform.rotation.w
                    ]
                )

                print(T_bt_ref)

                nu_bt_ref = np.zeros(6)
                dnu_bt_ref = np.zeros(6)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                if VERBOSE:
                    print(e)
                rate.sleep()
                continue
            
        else:
            T_bt_ref, nu_bt_ref, dnu_bt_ref = compute_sinusoid_pose_delta_reference(DELTA_NU, PERIOD_NU, (t - t0).to_sec())
            nu_bt_ref = np.zeros(6)
            dnu_bt_ref = np.zeros(6)

        msg = PoseTaskGoal()
        msg.header.stamp.secs = t.secs
        msg.header.stamp.nsecs = t.nsecs

        msg.pose.position.x = T_bt_ref.translation[0]
        msg.pose.position.y = T_bt_ref.translation[1]
        msg.pose.position.z = T_bt_ref.translation[2]
        q = pin.Quaternion(T_bt_ref.rotation)
        msg.pose.orientation.x = q.x
        msg.pose.orientation.y = q.y
        msg.pose.orientation.z = q.z
        msg.pose.orientation.w = q.w

        msg.twist.linear.x = nu_bt_ref[0]
        msg.twist.linear.y = nu_bt_ref[1]
        msg.twist.linear.z = nu_bt_ref[2]
        msg.twist.angular.x = nu_bt_ref[3]
        msg.twist.angular.y = nu_bt_ref[4]
        msg.twist.angular.z = nu_bt_ref[5]

        msg.acceleration.linear.x = dnu_bt_ref[0]
        msg.acceleration.linear.y = dnu_bt_ref[1]
        msg.acceleration.linear.z = dnu_bt_ref[2]
        msg.acceleration.angular.x = dnu_bt_ref[3]
        msg.acceleration.angular.y = dnu_bt_ref[4]
        msg.acceleration.angular.z = dnu_bt_ref[5]

        pub.publish(msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException as e:
        print(e)