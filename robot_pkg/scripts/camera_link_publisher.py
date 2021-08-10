#!/usr/bin/env python

import math
from math import sin, cos, pi

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3


x = 0.0
y = 0.0
th = 0.0


def callback(msg):
    global last_time, x, y, th
    global odom_pub, message, odom_broadcaster, current_time, last_time
    print("recibiendo msd = ", msg)
    vx  = 0
    vy  = 0
    vth = 0 

    current_time = rospy.Time.now()
    
    x  = 0
    y  = 0

    # since all odometry is 6DOF we'll need a quaternion created from yaw
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, 0)

    # first, we'll publish the transform over tf
    odom_broadcaster.sendTransform(
        (0.095, y, 0.314),
        odom_quat,
        current_time,
        "camera_link",
        "base_link"
    )

    # next, we'll publish the odometry message over ROS
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "base_link"

    # set the position
    odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

    # set the velocity
    odom.child_frame_id = "camera_link"
    odom.twist.twist = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))

    # publish the message
    print(" camera_link = ", odom)
    odom_pub.publish(odom)

    last_time = current_time
    #r.sleep()


def listener():

    global odom_pub, message, odom_broadcaster, current_time, last_time
    rospy.init_node('static_transformation_camera')
    odom_pub = rospy.Publisher("static_transformation_camera", Odometry, queue_size=50)
    odom_broadcaster = tf.TransformBroadcaster()
    current_time = rospy.Time.now()
    last_time = rospy.Time.now()
    message = Twist()
    rospy.Subscriber('static_transformation', Odometry, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()