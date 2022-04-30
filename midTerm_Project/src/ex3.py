#! /usr/bin/python3
from re import X
import numpy as np
from math import acos, asin, pi
from nav_msgs.msg import Odometry
import tf
import rospy
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock
import matplotlib.pyplot as plt
from config import *
from utils_turtlebot3 import *

np.random.seed(1)
rospy.init_node("node_ex3")
rate = rospy.Rate(100) #0.1s
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


def laser_measure():
    angle = get_imu_yaw()
    angle = (angle + 2*pi) if angle < 0 else angle
    angle = int(angle*180/pi)
    scan = rospy.wait_for_message("/scan",LaserScan)
    x = scan.ranges[360-angle-1] + np.random.normal(0, STANDARD_DEVIATION_MEASURE)
    y = scan.ranges[(630-angle-1) % 360] + np.random.normal(0, STANDARD_DEVIATION_MEASURE)
    return x, y

def estimate_position():
    x, y = laser_measure()
    x = WORLD_X_DIM - x
    y = (WORLD_Y_DIM - y) * -1
    return x, y

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
        # get initial position
        last_position = estimate_position()
        x.append(last_position[0])
        y.append(last_position[1])
        # add normal gaussian
        target_position = [np.random.normal(COORDINATES[i+1][0],STANDARD_DEVIATION),np.random.normal(COORDINATES[i+1][1],STANDARD_DEVIATION)]
        mover(last_position, target_position)
        if i!= len(COORDINATES)-2:
            input("Press any key...")
        
    last_position = estimate_position()
    x.append(last_position[0])
    y.append(last_position[1])
    
    fig, ax = plt.subplots()
    plot(np.array(x),np.array(y),ax, "REAL PATH")
    c = np.array(COORDINATES)
    plot(c[:,0],c[:,1],ax,"GROUND TRUTH")
    plt.legend()
    plt.show()


