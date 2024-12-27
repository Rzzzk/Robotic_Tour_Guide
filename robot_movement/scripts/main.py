#! /usr/bin/env python3

import rospy
from std_msgs.msg import String

from robot import Robot 

def main():

    #initialize ros node
    tour_guide = Robot()

    #initialize ros rate
    rate = rospy.Rate(10)

    #while user not exit from the program
    while not rospy.is_shutdown():

        #sleep for a time
        rate.sleep()


if __name__ == '__main__' :
    try:
        main()
    except rospy.ROSInterruptException:
        pass