#! /usr/bin/python3
import numpy as np
from math import acos, asin, pi


import rospy
from geometry_msgs.msg import Twist

COORDINATES = [(5, 0), (5, -3), (10, -3), (10, 0), (10, 3)]  #landmarks coordinates

def compute_angle(p1,p2):
    a = np.array(p1)
    b = np.array(p2)
 
    v_dist = np.subtract(b,a)
    v_norm = np.linalg.norm(v_dist)
 
    angle_1 = acos(v_dist[0]/v_norm)
    angle_2 = asin(v_dist[1]/v_norm)
 
 
 
    if (angle_1 >= 0 and angle_1 <= pi/2) and \
        (angle_2 >= 0 and angle_2 <= pi/2):
        return angle_1
    elif (angle_1 > pi/2 and angle_1<= pi) and \
        (angle_2 > 0 and angle_2 < pi/2):
        return angle_1
    elif (angle_1 > pi/2 and angle_1 < pi) and \
        (angle_2 > pi/-2 and angle_2 < 0):
        return pi/2*3 + angle_2
    elif (angle_1 > 0 and angle_1 <= pi/2) and \
        (angle_2 >= pi/-2 and angle_2 < 0):
        return 2*pi + angle_2





def mover(target_point, curr_pos=None):
    if(curr_pos is None):
        pass




