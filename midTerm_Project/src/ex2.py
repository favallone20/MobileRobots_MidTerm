#! /usr/bin/python3
import numpy as np
from math import acos, asin, pi
from nav_msgs.msg import Odometry
import tf
import rospy
from geometry_msgs.msg import Twist
from rosgraph_msgs.msg import Clock
import matplotlib.pyplot as plt

np.random.seed(1)
rospy.init_node("node_ex2")
COORDINATES = [(0,0), (0.7,0), (1,0.0), (1.5,0)]  #landmarks coordinates
ANGULAR_THRESHOLD = 0.001
POSITION_THRESHOLD = 0.01
STANDARD_DEVIATION = 0.1
LINEAR_SPEED = 0.1
ANGULAR_SPEED = 0.2
current_angle = 0
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


def get_position():
    odom = rospy.wait_for_message("/odom",Odometry)
    # In this code we assume to have the right angle of the robot
    x =  odom.pose.pose.position.x
    y = odom.pose.pose.position.y
    return round(x,2), round(y,2)

def get_yaw():
    odom = rospy.wait_for_message("/odom",Odometry)
    quaternion = (
    odom.pose.pose.orientation.x,
    odom.pose.pose.orientation.y,
    odom.pose.pose.orientation.z,
    odom.pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    return round(euler[2],2)

def get_time():
    clock = rospy.wait_for_message("/clock", Clock)
    time = clock.clock.secs + clock.clock.nsecs * 10**-9
    return time

def angular_movement(current_point, target_point):
    global current_angle
    angle_to_go = round(compute_angle(current_point,target_point),2)
    angle_to_go = angle_to_go if angle_to_go > current_angle else angle_to_go + 2*pi
    angle = abs(current_angle - angle_to_go)
    print("Current:" , current_point, "Target:", target_point)
    print("Angle to go:", angle_to_go, "Angle:", angle)
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
    current_angle = angle_to_go % (2*pi)

    
def mover(current_point, target_point):
    current_point = np.array(current_point)
    target_point = np.array(target_point)
    angular_movement(current_point, target_point)
    distance_vector = np.subtract(target_point, current_point)
    distance = np.linalg.norm(distance_vector)
    movement_time = distance/0.1
    twist = Twist()
    twist.linear.x = 0.1
    start_time = get_time()
    pub.publish(twist)
    while (get_time() - start_time < movement_time):
        pub.publish(twist)
        rate.sleep()    
    twist.linear.x=0
    pub.publish(twist)

if __name__=="__main__":
    
    target_position = get_position()
    x = []
    y = []
    for i in range(len(COORDINATES)-1):
        # get initial position
        last_position = target_position
        x.append(last_position[0])
        y.append(last_position[1])
        # compute distance from real points
        dist = np.subtract(COORDINATES[i+1],COORDINATES[i])
        # compute new point from the last position, in first iteration
        # last_position and new_position are COORDINATES[0] e COORDINATES[1]
        new_position = last_position+dist
        # add normal ga
        target_position = [np.random.normal(new_position[0],STANDARD_DEVIATION),np.random.normal(new_position[1],STANDARD_DEVIATION)]

        mover(last_position, target_position)
        
        if i!= len(COORDINATES)-2:
            input("Press any key...")
        rospy.loginfo(get_yaw())
        rospy.loginfo(get_position())
    
    x.append(target_position[0])
    y.append(target_position[1])
    
    c = np.array(COORDINATES)
    plt.scatter(c[:,0],c[:,1],label="true path")
    plt.scatter(x,y, label = "real path")
    plt.legend()
    plt.show()