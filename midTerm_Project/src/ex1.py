#! /usr/bin/python3
import numpy as np
from math import acos, asin, pi
from nav_msgs.msg import Odometry
import tf
import rospy
from geometry_msgs.msg import Twist

COORDINATES = [(0,0), (0.26,0), (0.5,0), (1,0), (1.5,0)]  #landmarks coordinates
rate = rospy.Rate(10) #0.1s
pub = rospy.Publisher("/cmd_vel",Twist,queue_size=1)


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



def get_yaw():
    odom = rospy.wait_for_message("/odom",Odometry)
    quaternion = (
    odom.pose.orientation.x,
    odom.pose.orientation.y,
    odom.pose.orientation.z,
    odom.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    return euler[2]

def mover(target_point, curr_point):
    angle = compute_angle(curr_point,target_point)
    yaw = get_yaw()
    # while yaw !=  angle:
    #     twist = Twist()
    #     twist.angular.z = 0.18
    #     pub.publish(twist)
    #     rate.sleep()
    #     yaw = get_yaw()
    #     print(yaw)


if __name__=="__main__":
    rospy.loginfo(get_yaw())







