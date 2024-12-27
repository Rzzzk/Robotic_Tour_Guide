#! /usr/bin/env python3

import rospy
from std_msgs.msg import String

from point_of_interests import POIx

from robot import Robot 

import time


def main():

    #initialize ros node
    tour_guide = Robot()

    #initialize ros rate
    rate = rospy.Rate(10)

    for poi in POIx:
        
        # publish the goal pose
        while not tour_guide.is_goal_accepted():
            tour_guide.publish_goal_pose(poi.map_pose)


        # wait for the robot to reach the goal
        while not tour_guide.is_goal_reached():
            tour_guide.publish_general_status(f"Moving to {poi.name}, point ID {poi.ID}") 

        # the robot has reached the goal
        tour_guide.publish_general_status(f"Reached {poi.name}, point ID {poi.ID}")
        time.sleep(10)


    #while user not exit from the program
    while not rospy.is_shutdown():

        #sleep for a time
        rate.sleep()


if __name__ == '__main__' :
    try:
        main()
    except rospy.ROSInterruptException:
        pass