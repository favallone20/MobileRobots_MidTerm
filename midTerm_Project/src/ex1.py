#! /usr/bin/python3
import numpy as np
from math import acos, asin, pi
from sensor_msgs.msg import Imu
import tf
import rospy
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock
import matplotlib.pyplot as plt
from config import *
from utils_turtlebot3 import *

rospy.init_node("node_ex1")
rate = rospy.Rate(100) #0.01s
pub = rospy.Publisher("/cmd_vel",Twist,queue_size=1)


def compute_angle(p1,p2):
    
    v_dist = np.subtract(p2,p1)
    v_norm = np.linalg.norm(v_dist)
 
    angle_1 = acos(v_dist[0]/v_norm)
    angle_2 = asin(v_dist[1]/v_norm)
    
    if (angle_1 >= 0 and angle_1 <= pi/2) and \
        (angle_2 >= 0 and angle_2 <= pi/2):
        angle_to_follow = angle_1
    elif (angle_1 > pi/2 and angle_1<= pi) and \
        (angle_2 > 0 and angle_2 < pi/2):
        angle_to_follow = angle_1
    elif (angle_1 > pi/2 and angle_1 < pi) and \
        (angle_2 > pi/-2 and angle_2 < 0):
        angle_to_follow = pi + abs(angle_2)
    elif (angle_1 > 0 and angle_1 <= pi/2) and \
        (angle_2 >= pi/-2 and angle_2 < 0):
        angle_to_follow = 2*pi + angle_2
    
    return angle_to_follow


def get_imu_yaw():
    imu = rospy.wait_for_message("/imu", Imu)
    quaternion = (
    imu.orientation.x,
    imu.orientation.y,
    imu.orientation.z,
    imu.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    return round(euler[2],2)

def get_time():
    clock = rospy.wait_for_message("/clock", Clock)
    time = clock.clock.secs + clock.clock.nsecs * 10**-9
    return time

def angular_movement(current_point, target_point):
    '''Rotates the robot toward the target point in a way that only
    linear movement along x axis of the robot reference frame is needed'''
    
    #Compute the current angle
    current_angle = get_imu_yaw()
    current_angle = current_angle if current_angle > 0 else 2*pi + current_angle
    
    #Compute destination angle
    angle_to_go = round(compute_angle(current_point,target_point),2)
    angle_to_go = angle_to_go if angle_to_go > current_angle else angle_to_go + 2*pi
    
    #Compute total movement angle 
    angle = abs(current_angle - angle_to_go)
    
    movement_time = angle/ANGULAR_SPEED
    twist = Twist()
    twist.angular.z = ANGULAR_SPEED
    start_time = get_time()
    pub.publish(twist)
    
    while (get_time() - start_time < movement_time):
        pub.publish(twist)
        rate.sleep()
    
    twist.angular.z = 0
    pub.publish(twist)

    
def mover(current_point, target_point):
    current_point = np.array(current_point)
    target_point = np.array(target_point)
    angular_movement(current_point, target_point)
    distance_vector = np.subtract(target_point, current_point)
    distance = np.linalg.norm(distance_vector)
    movement_time = distance/LINEAR_SPEED
    twist = Twist()
    twist.linear.x = LINEAR_SPEED
    start_time = get_time()
    pub.publish(twist)
    while (get_time() - start_time < movement_time):
        pub.publish(twist)
        rate.sleep()    
    twist.linear.x=0
    pub.publish(twist)


if __name__=="__main__":
    
    x = []
    y = []
    
    for i in range(len(COORDINATES)-1):
        mover(COORDINATES[i], COORDINATES[i+1])
        x.append(COORDINATES[i][0])
        y.append(COORDINATES[i][1])
        if i!= len(COORDINATES)-2:
            input("Press any key...")
    
    fig, ax = plt.subplots()
    x.append(COORDINATES[i][0])
    y.append(COORDINATES[i][1])
    plot(np.array(x),np.array(y),ax, "REAL PATH")
    
    c = np.array(COORDINATES)
    plot(c[:,0],c[:,1],ax,"GROUND TRUTH")
    plt.legend()
    plt.show()
            







