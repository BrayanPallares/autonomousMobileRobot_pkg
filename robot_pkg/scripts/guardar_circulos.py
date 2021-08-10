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
import numpy as np 
from numpy import save
import pygame

px_array        = []
py_array        = []
px1_array        = []
py1_array        = []

leyendo_serial = 1
pygame.init()

dimensiones = [480, 600]
pantalla = pygame.display.set_mode(dimensiones)
pygame.display.set_caption("Animaciones")


def callback_odom(msg):
    global px_array, py_array, leyendo_serial
    px_array.append(msg.pose.pose.position.x)
    py_array.append(msg.pose.pose.position.y)
    if leyendo_serial == 1:
        save('px_array.npy', px_array )
        save('py_array.npy', py_array )
        print("guardado")
    else:
        print("ya no se guarda")

    for evento in pygame.event.get():
        if evento.type == pygame.KEYDOWN:
            if evento.key == pygame.K_RIGHT:    
                leyendo_serial = 2

def callback_odom_wheels(msg):
    global px1_array, py1_array, leyendo_serial
    px1_array.append(msg.pose.pose.position.x)
    py1_array.append(msg.pose.pose.position.y)
    if leyendo_serial == 1:
        save('px1_array.npy', px1_array )
        save('py1_array.npy', py1_array )
        print("guardado")
    else:
        print("ya no se guarda")

    for evento in pygame.event.get():
        if evento.type == pygame.KEYDOWN:
            if evento.key == pygame.K_RIGHT:    
                leyendo_serial = 2


def listener():
    rospy.init_node('guardar_circulos')
    rospy.Subscriber('odom_wheels', Odometry, callback_odom_wheels)
    rospy.Subscriber('odom', Odometry, callback_odom)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()