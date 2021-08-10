#!/usr/bin/env python
# license removed for brevity

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from sensor_msgs.msg import Joy
import serial
import struct
import os
import math
from math import sin, cos, pi
import tf
from nav_msgs.msg import Odometry

start_button = 0
back_button = 0
x = 0.0
y = 0.0
th = 0.0



def callback_joy(msg):
    global start_button, back_button
    print("start = ", msg.buttons[7])
    print("back  = ", msg.buttons[6])
    start_button = msg.buttons[7]
    back_button  = msg.buttons[6]


def callback(msg):
	global start_button, back_button
	global last_time, x, y, th
	global odom_pub, message, odom_broadcaster, current_time, last_time
	with serial.Serial('/dev/ttyACM1', 115200, timeout=0.0025) as ser:
		try:
			print("Leyendo serial.....")

			s = ser.read(112)        # read up to ten bytes (timeout)
			print("leido de serial = ", s)
			px = struct.unpack('d',s[0:8])[0]
			py = struct.unpack('d',s[8:16])[0]
			rz = struct.unpack('d',s[16:24])[0]

			vx_read = struct.unpack('d',s[24:32])[0]
			vy_read = struct.unpack('d',s[32:40])[0]
			wz_read = struct.unpack('d',s[40:48])[0]

			velWheel1 = struct.unpack('d',s[48:56])[0]
			velWheel2 = struct.unpack('d',s[56:64])[0]
			velWheel3 = struct.unpack('d',s[64:72])[0]
			velWheel4 = struct.unpack('d',s[72:80])[0]

			control1 = struct.unpack('d',s[80:88])[0]
			control2 = struct.unpack('d',s[88:96])[0]
			control3 = struct.unpack('d',s[96:104])[0]
			control4 = struct.unpack('d',s[104:112])[0]

			vx  = vx_read
			vy  = vy_read
			vth = wz_read

			current_time = rospy.Time.now()

			# compute odometry in a typical way given the velocities of the robot

			dt = (current_time - last_time).to_sec()
			delta_x = (vx * cos(th) - vy * sin(th)) * dt
			delta_y = (vx * sin(th) + vy * cos(th)) * dt
			delta_th = vth * dt

			x += delta_x
			y += delta_y
			th += delta_th

			if start_button != 0:
				x  = 0
				y  = 0
				th = 0


			# since all odometry is 6DOF we'll need a quaternion created from yaw
			odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

			#uncomment to use without camera T
			#odom_broadcaster.sendTransform(
			#(x, y, 0.),
			#odom_quat,
			#current_time,
			#"base_footprint",
			#"odom"
			#)

			# next, we'll publish the odometry message over ROS
			odom = Odometry()
			odom.header.stamp = current_time
			odom.header.frame_id = "odom"

			# set the position
			odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

			# set the velocity
			odom.child_frame_id = "base_footprint"
			odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

			# publish the message
			print(" odom = ", odom)
			odom_pub.publish(odom)

			last_time = current_time
			#r.sleep()
			

		except:
			print("no hay nada que leer")


		message = struct.pack('dddddd',msg.linear.x, -msg.linear.y, -msg.angular.z, start_button, back_button, 9912399)                
		ser.write(message)
		print("escrito correctamente")
		ser.close()




def listener():
    global start_button, back_button
    global odom_pub, message, odom_broadcaster, current_time, last_time

    rospy.init_node('odometry_publisher')
    odom_pub = rospy.Publisher("odom_wheels", Odometry, queue_size=50)
    odom_broadcaster = tf.TransformBroadcaster()
    current_time = rospy.Time.now()
    last_time = rospy.Time.now()
    message = Twist()
    rospy.Subscriber('cmd_vel', Twist, callback)
    #rospy.Subscriber('joy', Joy, callback_joy)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()