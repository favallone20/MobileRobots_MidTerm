#! /usr/bin/python3
import csv
from tokenize import Double, PseudoExtras

from rosdep2 import RosdepLookup
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
import time

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

n_reached = 0

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
        
    start = MoveBaseGoal()
    start.target_pose.header.frame_id="map"
    start.target_pose.header.stamp = rospy.Time.now()
    start.target_pose.pose.position.x = float(rows[0][0].split()[0])
    start.target_pose.pose.position.y = float(rows[0][0].split()[1])
    start.target_pose.pose.orientation.w = 1

    end = MoveBaseGoal()
    end.target_pose.header.frame_id="map"
    end.target_pose.header.stamp = rospy.Time.now()
    end.target_pose.pose.position.x = float(rows[1][0].split()[0])
    end.target_pose.pose.position.y = float(rows[1][0].split()[1])
    end.target_pose.pose.orientation.w = 1

    print("partenza")
    client.send_goal(start)
    wait = client.wait_for_result()
    print("raggiunto")
    time.sleep(1)

    for i in range(2, len(rows)):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = float(rows[i][0].split()[0])
        goal.target_pose.pose.position.y = float(rows[i][0].split()[1])
        goal.target_pose.pose.orientation.w = 1
        client.send_goal(goal)
        wait = client.wait_for_result()

        time.sleep(1)


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





        
    
