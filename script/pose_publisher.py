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


from panda_torque_mpc.msg import PoseTaskGoal

LISTEN_TO_TF = False
TOPIC_POSE_PUBLISHED = 'ee_pose_ref'

FREQ = 100
DT = 1/FREQ
DELAY_AVOID_EXTRAP = 0.05
VERBOSE = True
# we want T_wc
camera_pose_frame = "camera_pose_frame";  # moving "camera=c" frame
world_frame = "camera_odom_frame";  # static inertial "world=w" frame



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
    dx_r =     -w * a * np.sin(w*t)
    ddx_r = -w**2 * a * np.cos(w*t)

    x_r_local = pin.exp(nu)

    return x_r_local, dx_r, ddx_r




# DELTA_NU = np.array([
#     0.0, 0.0, 0.0, 
#     0.0, 0.0, 0.0
# ])

# DELTA_NU = np.array([
#     0.04, 0.04, 0.04, 
#     0.0, 0.0, 0.0
# ])

DELTA_NU = np.array([
    0.09, 0.09, 0.09, 
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

    # receives tf2 messages from the /tf topic and buffers them for 10 second (by default)
    tfBuffer = tf2_ros.Buffer()

    tf2_ros.TransformListener(tfBuffer)

    dt_to_fill_the_buffer = 1.0
    print(f'Filling tf transform buffer for {dt_to_fill_the_buffer} seconds')
    rospy.sleep(dt_to_fill_the_buffer)

    t0 = rospy.Time.now()
    while not rospy.is_shutdown():
        t = rospy.Time.now()

        print('t', t)

        if LISTEN_TO_TF:
            try:
                # Add delay to make sure we are not extrapolating the tf lookup
                # Documentation is not clear about which transformation is retrieved by lookup_transform (target/source or source/target)
                # After testing with T265 node, it seems it is T_wc, s.t. 
                # w_vec = T_wc * c_vec
                T_wc = tfBuffer.lookup_transform(world_frame, camera_pose_frame, t - rospy.Duration(DELAY_AVOID_EXTRAP))
                x_r_local = pin.XYZQUATToSE3(
                    [
                        T_wc.transform.translation.x, T_wc.transform.translation.y, T_wc.transform.translation.z,
                        T_wc.transform.rotation.x, T_wc.transform.rotation.y, T_wc.transform.rotation.z, T_wc.transform.rotation.w
                    ]
                )

                dx_r = np.zeros(6)
                ddx_r = np.zeros(6)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                if VERBOSE:
                    print(e)
                rate.sleep()
                continue
            
        else:
            x_r_local, dx_r, ddx_r = compute_sinusoid_pose_delta_reference(DELTA_NU, PERIOD_NU, (t - t0).to_sec())

        msg = PoseTaskGoal()
        msg.header.stamp.secs = t.secs
        msg.header.stamp.nsecs = t.nsecs

        msg.pose.position.x = x_r_local.translation[0]
        msg.pose.position.y = x_r_local.translation[1]
        msg.pose.position.z = x_r_local.translation[2]
        q = pin.Quaternion(x_r_local.rotation)
        msg.pose.orientation.x = q.x
        msg.pose.orientation.y = q.y
        msg.pose.orientation.z = q.z
        msg.pose.orientation.w = q.w

        msg.twist.linear.x = dx_r[0]
        msg.twist.linear.y = dx_r[1]
        msg.twist.linear.z = dx_r[2]
        msg.twist.angular.x = dx_r[3]
        msg.twist.angular.y = dx_r[4]
        msg.twist.angular.z = dx_r[5]

        msg.acceleration.linear.x = ddx_r[0]
        msg.acceleration.linear.y = ddx_r[1]
        msg.acceleration.linear.z = ddx_r[2]
        msg.acceleration.angular.x = ddx_r[3]
        msg.acceleration.angular.y = ddx_r[4]
        msg.acceleration.angular.z = ddx_r[5]

        pub.publish(msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException as e:
        print(e)