#! /usr/bin/python3
import csv
from tokenize import Double, PseudoExtras
import tf
from rosdep2 import RosdepLookup
import rospy
from geometry_msgs.msg import PoseStamped,Quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
import time

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from numpy import subtract
from numpy.linalg import norm
from math import atan2,acos,asin,pi
n_reached = 0

def compute_angle(start_point,end_point):
    v_diff = subtract(end_point,start_point)
    # v_norm = norm(v_diff)
    v_diff[0] += 0.00001
    orientation = atan2(v_diff[1],v_diff[0])
    return orientation

# def compute_angle(p1,p2):
#     ''''Computes angle between two vectors (angle in range [0, 2*pi])'''
    
#     v_dist = subtract(p2,p1)
#     v_norm = norm(v_dist)
#     angle_1 = acos(v_dist[0]/v_norm)
#     angle_2 = asin(v_dist[1]/v_norm)
    
#     if (angle_1 >= 0 and angle_1 <= pi/2) and \
#         (angle_2 >= 0 and angle_2 <= pi/2):
#         angle_to_follow = angle_1
#     elif (angle_1 > pi/2 and angle_1<= pi) and \
#         (angle_2 > 0 and angle_2 < pi/2):
#         angle_to_follow = angle_1
#     elif (angle_1 > pi/2 and angle_1 < pi) and \
#         (angle_2 > pi/-2 and angle_2 < 0):
#         angle_to_follow = pi + abs(angle_2)
#     elif (angle_1 > 0 and angle_1 <= pi/2) and \
#         (angle_2 >= pi/-2 and angle_2 < 0):
#         angle_to_follow = 2*pi + angle_2
    
#     return angle_to_follow


def callback(data: GoalStatusArray):
    global n_reached 
    length = len(data.status_list)
    for s in data.status_list:
        if s.status == 3:
            n_reached = n_reached+1
            if n_reached == 5:
                print("raggiunto")
            
        if s.status == 1:
            n_reached = 0
            print("non raggiunto")
    
if __name__=="__main__":
    rospy.init_node("read_trajectory")
    # rate = rospy.Rate(1)
    # pubGoal = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=1000)

    # MoveBaseclient("move_base")

    # #while not rospy.is_shutdown():
    # time.sleep(1)
    # goal = PoseStamped()
    # goal.header.frame_id="map"
    # goal.pose.position.x = 0.5
    # goal.pose.position.y = -1.5
    # goal.pose.orientation.w = -1.0
    # pubGoal.publish(goal)
    # print("pubblico")
    # rate.sleep()


    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    print("ednytro")
    client.wait_for_server(rospy.Duration(5))
    print("sono uscito")

    file_name = rospy.get_param("file_name")
    with open(file_name) as f:
        csvreader = csv.reader(f)
        rows = []
        for row in csvreader:
            rows.append(row)
    
    print(rows[0][0], rows[1][0])
        
    start_x = float(rows[0][0].split()[0])
    start_y = float(rows[0][0].split()[1])

    start = MoveBaseGoal()
    start.target_pose.header.frame_id="map"
    start.target_pose.header.stamp = rospy.Time.now()
    start.target_pose.pose.position.x = start_x
    start.target_pose.pose.position.y = start_y
    start.target_pose.pose.orientation.w = 1

    end_x = float(rows[1][0].split()[0])
    end_y = float(rows[1][0].split()[1])

    end = MoveBaseGoal()
    end.target_pose.header.frame_id="map"
    end.target_pose.header.stamp = rospy.Time.now()
    end.target_pose.pose.position.x = end_x
    end.target_pose.pose.position.y = end_y
    end.target_pose.pose.orientation.w = 1

    print("partenza")
    client.send_goal(start)
    wait = client.wait_for_result()
    print("raggiunto")
    time.sleep(0.01)

    for i in range(2, len(rows)):
        goal = MoveBaseGoal()

        goal_x = float(rows[i][0].split()[0])
        goal_y = float(rows[i][0].split()[1])

        if i < (len(rows)-1):
            next_goal_x = float(rows[i+1][0].split()[0])
            next_goal_y = float(rows[i+1][0].split()[1])
            yaw = compute_angle([goal_x,goal_y],[next_goal_x,next_goal_y])
            quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
        else:
            yaw = compute_angle([goal_x,goal_y],[end_x,end_y])
            quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = goal_x
        goal.target_pose.pose.position.y = goal_y
        goal.target_pose.pose.orientation.x = quaternion[0]
        goal.target_pose.pose.orientation.y = quaternion[1]
        goal.target_pose.pose.orientation.z = quaternion[2]
        goal.target_pose.pose.orientation.w = quaternion[3]
        client.send_goal(goal)
        wait = client.wait_for_result()
        time.sleep(0.01)
        

    client.send_goal(end)
    wait = client.wait_for_result()
    print("raggiunto")
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        print(client.get_result())




    # goal = MoveBaseGoal()
    # goal.target_pose.header.frame_id = "map"
    # goal.target_pose.header.stamp = rospy.Time.now()
    # goal.target_pose.pose.position.x = -0.5
    # goal.target_pose.pose.position.y = 1.5
    # goal.target_pose.pose.orientation.w = 1

   
    # pubInitialPose = rospy.Publisher("initialpose", PoseWithCovarianceStamped)

    #subMoveStatus = rospy.Subscriber("move_base/status", GoalStatusArray, callback)

    # start = PoseWithCovarianceStamped()
    # start.pose.pose.position.x = rows[0][0].split()[0]
    # start.pose.pose.position.y = rows[0][0].split()[1]
    # start.pose.pose.orientation.w = 1
    # pubInitialPose.publish(start)





        
    
