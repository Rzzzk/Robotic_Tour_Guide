#!/usr/bin/env python3

# Importing the required libraries
import point_of_interests
from map_pose import MapPose
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped


# Class definition
class Robot:


    # Constructor
    def __init__(self):
        self.map_pose = MapPose(0.0, 0.0, 0.0, 0.0)
        self.home_pose = MapPose(0.0, 0.0, 0.0, 0.0)
        self.goal_pose = MapPose(0.0, 0.0, 0.0, 0.0)
        self.goal_accepted = False
        self.goal_reached = False


        # Initialize the node
        rospy.init_node('robot_node')

        # Initialize the subscriber
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.update_map_pose)

        # Initialize the publisher
        self.pub_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)



    # Callback function to update the map pose
    def update_map_pose(self, msg):
        pose = msg.pose.pose
        x = pose.position.x
        y = pose.position.y
        z = pose.orientation.z
        w = pose.orientation.w
        self.map_pose.set_pose(x, y, z, w)
        rospy.loginfo(f"Updated pose: x={x}, y={y}, z={z}, w={w}")


    # Constructor
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
