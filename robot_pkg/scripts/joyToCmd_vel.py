#!/usr/bin/env python
# license removed for brevity

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import serial
import struct
import os

#def callback(data):
#    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
def callback(msg):
    global message
    print("message read = ", msg)
    print("start = ", msg.buttons[7])
    print("back  = ", msg.buttons[6])
    turbo = msg.axes[5] + 1
    turbo2 = (turbo * 2) + 0.5
    
    message.linear.x  = msg.axes[1]/turbo2
    message.linear.y  = msg.axes[0]/turbo2
    message.linear.z  = 0.0
    message.angular.x = 0.0
    message.angular.y = 0.0
    message.angular.z = msg.axes[3]/turbo2
    clear = lambda: os.system('clear')
    #clear()
    pub.publish(message)


def listener():

    global pub, message
    message = Twist()
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=50)
    rospy.init_node('joy2cmdvel', anonymous=True)
    rospy.Subscriber('joy', Joy, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()