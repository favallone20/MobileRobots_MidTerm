#! /usr/bin/python3
import csv
from tokenize import Double, PseudoExtras

from rosdep2 import RosdepLookup
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from actionlib_msgs.msg import GoalStatusArray, GoalStatus


n_reached = 0

def callback(data: GoalStatusArray):
    global n_reached 
    length = len(data.status_list)
    for s in data.status_list:
        if s.status == 3:
            n_reached = n_reached+1
            if n_reached == 2:
                print("raggiunto")
            
        if s.status == 1:
            n_reached = 0
            print("non raggiunto")
    
if __name__=="__main__":
    rospy.init_node("read_trajectory")
    
    file_name = rospy.get_param("file_name")
    with open(file_name) as f:
        csvreader = csv.reader(f)
        rows = []
        for row in csvreader:
            rows.append(row)
            
    print(rows)

    pubGoal = rospy.Publisher("move_base_simple/goal", PoseStamped,queue_size=1)
    # pubInitialPose = rospy.Publisher("initialpose", PoseWithCovarianceStamped)

    # subMoveStatus = rospy.Subscriber("move_base/status", GoalStatusArray, callback)

    # start = PoseWithCovarianceStamped()
    # start.pose.pose.position.x = rows[0][0].split()[0]
    # start.pose.pose.position.y = rows[0][0].split()[1]
    # start.pose.pose.orientation.w = 1
    # pubInitialPose.publish(start)

    goal = PoseStamped()
    goal.header.frame_id="map"
    goal.pose.position.x = 1
    goal.pose.position.y = 0
    goal.pose.orientation.w = 1.0
    pubGoal.publish(goal)
    print("ho pubblicato")
    
    while(True):
        pass





        
    
