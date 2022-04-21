#! /usr/bin/python3
from matplotlib.contour import QuadContourSet
import numpy as np
from math import acos, asin, pi
from nav_msgs.msg import Odometry
import tf
from tf import TransformListener
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped, Quaternion

rospy.init_node("node_ex3")
COORDINATES = [(0,0), (0.7,0), (1,0.0), (1.5,0)]  #landmarks coordinates
ANGULAR_THRESHOLD = 0.001
POSITION_THRESHOLD = 0.01
STANDARD_DEVIATION = 0.05
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

# def get_position():
#     odom = rospy.wait_for_message("/odom",Odometry)
#     # In this code we assume to have the right angle of the robot
#     x =  odom.pose.pose.position.x
#     y = odom.pose.pose.position.y
#     return round(x,2), round(y,2)

# def get_yaw():
#     odom = rospy.wait_for_message("/odom",Odometry)
#     quaternion = (
#     odom.pose.pose.orientation.x,
#     odom.pose.pose.orientation.y,
#     odom.pose.pose.orientation.z,
#     odom.pose.pose.orientation.w)
#     euler = tf.transformations.euler_from_quaternion(quaternion)
#     return round(euler[2],2)

def from_laser_to_odom(laser_point):
    # laser point is like [x,y,yaw]
    pose = PoseStamped()
    pose.header.frame_id="base_scan"
    quaternion = tf.transformations.quaternion_from_euler(0, 0, laser_point[2])
    pose.pose.position.x = laser_point[0]
    pose.pose.position.y = laser_point[1]
    pose.pose.orientation = Quaternion(*quaternion)
    print("----------------")
    print(pose.pose.position.x,pose.pose.position.y)
    print("----------------")
    
    print(tf_listener.frameExists("odom"), tf_listener.frameExists("base_scan"))
    if tf_listener.frameExists("/odom") and tf_listener.frameExists("/base_scan"):
        pose_in_odom = tf_listener.transformPose("/odom",pose)

    print("----------------")
    print(pose_in_odom.pose.position.x,pose_in_odom.pose.position.y)
    print("----------------")
    return pose_in_odom

    
    

def estimate_position():
    scan = rospy.wait_for_message("/scan",LaserScan)

    odom_front = from_laser_to_odom([scan.ranges[0],0,0])
    # odom_left = from_laser_to_odom([0,scan.ranges[90],pi/2])
    # odom_rear = from_laser_to_odom([scan.ranges[180]*-1,0,pi])
    # odom_right = from_laser_to_odom([0,scan.ranges[270]*-1,pi/-2])

  



# def angular_movement(current_point, target_point):
#     angle = round(compute_angle(current_point,target_point),2)
#     twist = Twist()
#     while abs(get_yaw()-angle) > ANGULAR_THRESHOLD:
#         twist.angular.z = 0.2
#         pub.publish(twist)
#         rate.sleep()
#     twist.angular.z = 0
#     pub.publish(twist)
    
# def mover(current_point, target_point):
#     current_point = np.array(current_point)
#     target_point = np.array(target_point)
#     position = np.array(get_position())
#     angular_movement(current_point, target_point)
#     twist = Twist()
#     index = np.argmax(np.abs(position-target_point))
    
#     while(abs(get_position()[index]-target_point[index])>POSITION_THRESHOLD):
#         twist.linear.x = 0.1
#         pub.publish(twist)
#         rate.sleep()
#     rospy.loginfo(get_position())
#     twist.linear.x=0
#     pub.publish(twist)



if __name__=="__main__":
    estimate_position()
    # target_position = get_position()
    # for i in range(len(COORDINATES)-1):
    #     # get initial position
    #     last_position = target_position
    #     # compute distance from real points
    #     dist = np.subtract(COORDINATES[i+1],COORDINATES[i])
    #     # compute new point from the last position, in first iteration
    #     # last_position and new_position are COORDINATES[0] e COORDINATES[1]
    #     new_position = last_position+dist
    #     # add normal ga
    #     target_position = [np.random.normal(new_position[0],STANDARD_DEVIATION),np.random.normal(new_position[1],STANDARD_DEVIATION)]

    #     mover(last_position, target_position)
        
    #     if i!= len(COORDINATES)-2:
    #         input("Press any key...")
    #     rospy.loginfo(get_yaw())
    #     rospy.loginfo(get_position())