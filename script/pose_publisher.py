#!/usr/bin/env python

import numpy as np
import pinocchio as pin
import rospy
import tf2_ros


from panda_torque_mpc.msg import PoseTaskGoal

LISTEN_TO_TF = True

FREQ = 30
DT = 1/FREQ
DELAY = 0.01
VERBOSE = True
reference_frame = "camera_odom_frame"; 
pose_frame = "camera_link"; 


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




DELTA_NU = np.array([
    0.09, 0.09, 0.09, 
    0.0, 0.0, 0.0
])

PERIOD_NU = np.array([
    4.0, 4.0, 4.0, 
    5.0, 5.0, 5.0
])


def talker():
    pub = rospy.Publisher('target_pose', PoseTaskGoal, queue_size=10)
    
    rospy.init_node('pose_publisher', anonymous=False)
    rate = rospy.Rate(FREQ)

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    t0 = rospy.Time.now()
    while not rospy.is_shutdown() and not t0:
        t0 = rospy.Time.now()
        rate.sleep()
    
    while not rospy.is_shutdown():
        t = rospy.Time.now()

        print('t')
        print(t)

        if LISTEN_TO_TF:
            try:
                # add delay to make sure we are not extrapolating the tf lookup
                trans = tfBuffer.lookup_transform(reference_frame, pose_frame, t - rospy.Duration(DELAY))
                x_r_local = pin.XYZQUATToSE3(
                    [
                        trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z,
                        trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w
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