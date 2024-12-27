#!/usr/bin/env python3

# Importing the required libraries
import point_of_interests
from map_pose import MapPose

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray


import time


# Class definition
class Robot:


    # Constructor
    def __init__(self):
        
        # Initialize the map pose
        self.goal_pose = MapPose(0.0, 0.0, 0.0, 0.0)
        self.current_pse = MapPose(0.0, 0.0, 0.0, 0.0)


        self.goal_accepted = False
        self.goal_reached = False
        self.general_status = "Idle"



        # Initialize the node
        rospy.init_node('robot_node')

        # Initialize the subscriber
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.update_map_pose)

        # Initialize the publisher
        self.pub_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

        # Initialize the publisher
        self.pub_status = rospy.Publisher('/robot_status', String, queue_size=10)

        # Initialize the subscriber for the move_base status
        rospy.Subscriber('/move_base/status', GoalStatusArray, self.update_goal_status)

        # Wait for the node to initialize        
        time.sleep(2)



    # Callback function to update the map pose
    def update_map_pose(self, msg):
        pose = msg.pose.pose
        x = pose.position.x
        y = pose.position.y
        z = pose.orientation.z
        w = pose.orientation.w
        self.current_pse.set_pose(x, y, z, w)
        rospy.loginfo(f"Updated pose:")
        rospy.loginfo(f"x={x}, y={y}")
        rospy.loginfo(f"z={z}, w={w}")
    

    # Callback function to update the goal status
    def update_goal_status(self, msg):

        # Check if the status_list is not empty
        if msg.status_list:
            
            # Process the latest goal status
            latest_status = msg.status_list[-1].status
            
            if latest_status == 3:  # Goal reached
                self.goal_reached = True
                self.goal_accepted = False
                rospy.loginfo("Goal reached")
            
            elif latest_status == 1:  # Goal accepted
                self.goal_accepted = True
                self.goal_reached = False
                rospy.loginfo("Goal accepted")
            
            else:
                self.goal_accepted = False
                self.goal_reached = False
                rospy.loginfo(f"Other goal status: {latest_status}")


    # Function to check if the goal is accepted
    def is_goal_accepted(self):
        return self.goal_accepted

    # Function to check if the goal is reached   
    def is_goal_reached(self):
        return self.goal_reached

    # Function to get the current map pose
    def set_general_status(self, status)->String:
        self.general_status = status
        self.pub_goal.publish(status)
    

    # Function to get the current map pose
    def get_current_pose(self):
        return self.current_pse
    
    # Function to publish the general status
    def publish_general_status(self, status):
        self.general_status = status
        self.pub_status.publish(status)


    # Function to publish the goal pose
    def publish_goal_pose(self, gaol)->MapPose:

        self.goal_pose = gaol

        # Create a goal pose message
        goal_pose = PoseStamped()
        goal_pose.header.stamp = rospy.Time.now()
        goal_pose.header.frame_id = "map"

        # Set the goal pose
        goal_pose.pose.position.x = gaol.x
        goal_pose.pose.position.y = gaol.y
        goal_pose.pose.orientation.z = gaol.z
        goal_pose.pose.orientation.w = gaol.w
        
        # Publish the goal pose
        self.pub_goal.publish(goal_pose)
