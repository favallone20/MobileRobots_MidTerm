#! /usr/bin/python3
from re import X
from matplotlib.contour import QuadContourSet
import numpy as np
from math import acos, asin, pi
from nav_msgs.msg import Odometry
import tf
from tf import TransformListener
import rospy
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import Twist, PoseStamped, Quaternion

rospy.init_node("node_ex3")
COORDINATES = [(0,0), (0.7,0), (1,0.0), (1.5,0)]  #landmarks coordinates
ANGULAR_THRESHOLD = 0.001
POSITION_THRESHOLD = 0.01
STANDARD_DEVIATION = 0.05
WORLD_X_DIM = 1.91
WORLD_Y_DIM = 1.91
rate = rospy.Rate(100) #0.1s
pub = rospy.Publisher("/cmd_vel",Twist,queue_size=1)
tf_listener = TransformListener()

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
        angle_to_follow = pi/2*3 + angle_2
    elif (angle_1 > 0 and angle_1 <= pi/2) and \
        (angle_2 >= pi/-2 and angle_2 < 0):
        angle_to_follow = 2*pi + angle_2
    
    if angle_to_follow > pi:
        angle_to_follow = angle_to_follow-2*pi
    
    return angle_to_follow

def get_odom_position():
    odom = rospy.wait_for_message("/odom",Odometry)
    # In this code we assume to have the right angle of the robot
    x =  odom.pose.pose.position.x
    y = odom.pose.pose.position.y
    return round(x,2), round(y,2)

def get_odom_yaw():
    odom = rospy.wait_for_message("/odom",Odometry)
    quaternion = (
    odom.pose.pose.orientation.x,
    odom.pose.pose.orientation.y,
    odom.pose.pose.orientation.z,
    odom.pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    return round(euler[2],2)

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
    print(angle)
    x = scan.ranges[360-angle-1]
    y = scan.ranges[(630-angle-1) % 360]  # 360 + 270, mi porto in zero e poi shift di 270 gradi
    return x, y

def estimate_position():
    x, y = laser_measure()
    x = WORLD_X_DIM - x
    y = (WORLD_Y_DIM - y) * -1
    return x, y

def angular_movement(current_point, target_point):
    angle = round(compute_angle(current_point,target_point),2)
    twist = Twist()
    while abs(get_odom_yaw()-angle) > ANGULAR_THRESHOLD:
        twist.angular.z = 0.2
        pub.publish(twist)
        rate.sleep()
    twist.angular.z = 0
    pub.publish(twist)
    
def mover(current_point, target_point):
    current_point = np.array(current_point)
    target_point = np.array(target_point)
    position = np.array(get_odom_position())
    angular_movement(current_point, target_point)
    twist = Twist()
    index = np.argmax(np.abs(position-target_point))
    while(abs(get_odom_position()[index]-target_point[index])>POSITION_THRESHOLD):
        twist.linear.x = 0.1
        pub.publish(twist)
        rate.sleep()
    rospy.loginfo(get_odom_position())
    twist.linear.x=0
    pub.publish(twist)

if __name__=="__main__":
    for i in range(len(COORDINATES)-1):
        # get initial position
        last_position = estimate_position()
        # add normal gaussian
        target_position = [np.random.normal(COORDINATES[i+1][0],STANDARD_DEVIATION),np.random.normal(COORDINATES[i+1][1],STANDARD_DEVIATION)]
        mover(last_position, target_position)
        if i!= len(COORDINATES)-2:
            input("Press any key...")
        rospy.loginfo(get_imu_yaw())
        rospy.loginfo(estimate_position())