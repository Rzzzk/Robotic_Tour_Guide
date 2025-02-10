#!/usr/bin/env python3

"""
Differential Drive Robot Odometry Calculator
Calculates robot position and orientation from encoder ticks
Handles encoder overflow and direction changes
Publishes odometry data and tf transforms
"""

import rospy
import tf2_ros
import tf_conversions
from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion, Point
import math

class OdometryCalculator:
    def __init__(self):
        """Initialize the odometry calculator with robot parameters and ROS setup"""
        # Initialize ROS node
        rospy.init_node('odometry_calculator')
        
        # Robot physical parameters - ADJUST THESE FOR YOUR ROBOT
        self.WHEEL_RADIUS = 0.0325  # Wheel radius in meters
        self.WHEEL_BASE = 0.295     # Distance between wheels in meters
        self.TICKS_PER_REVOLUTION = 1440  # Encoder ticks per wheel revolution
        
        # Constants for 32-bit integer overflow detection
        self.INT32_MAX = 2**31 - 1  # Maximum value for 32-bit signed integer
        self.INT32_MIN = -2**31     # Minimum value for 32-bit signed integer
        self.OVERFLOW_THRESHOLD = 2**30  # Threshold to detect overflow (half of max value)
        
        # Odometry state variables
        self.x = 0.0          # Robot's X position in meters
        self.y = 0.0          # Robot's Y position in meters
        self.theta = 0.0      # Robot's orientation in radians
        
        # Variables for tick tracking and overflow handling
        self.last_left_ticks = 0    # Previous left encoder reading
        self.last_right_ticks = 0   # Previous right encoder reading
        self.total_left_ticks = 0   # Accumulated left ticks including overflows
        self.total_right_ticks = 0  # Accumulated right ticks including overflows
        self.first_callback = True  # Flag for initializing first encoder reading
        
        # Set up ROS publishers and subscribers
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=10)
        self.encoder_sub = rospy.Subscriber('encoder_ticks', Int32MultiArray, 
                                          self.encoder_callback)
        
        # Set up TF2 broadcaster for coordinate transforms
        self.br = tf2_ros.TransformBroadcaster()
        
        # Create timer for publishing odometry at fixed rate (20Hz)
        self.timer = rospy.Timer(rospy.Duration(0.05), self.publish_odometry)

    def handle_overflow(self, current_tick, last_tick):
        """
        Handle encoder tick overflow for a single encoder
        
        Args:
            current_tick (int): Current encoder tick reading
            last_tick (int): Previous encoder tick reading
            
        Returns:
            int: True tick delta accounting for potential overflow
        """
        # Calculate simple difference
        delta = current_tick - last_tick
        
        # Check if difference exceeds overflow threshold
        if abs(delta) > self.OVERFLOW_THRESHOLD:
            if delta < 0:  # Overflow in positive direction
                # Calculate true difference when counter goes from MAX to MIN
                delta = (self.INT32_MAX - last_tick) + (current_tick - self.INT32_MIN) + 1
            else:  # Overflow in negative direction
                # Calculate true difference when counter goes from MIN to MAX
                delta = (self.INT32_MIN - last_tick) + (current_tick - self.INT32_MAX) - 1
                
        return delta
        
    def encoder_callback(self, msg):
        """
        Process encoder ticks and update odometry
        
        Args:
            msg (Int32MultiArray): Message containing left and right encoder ticks
        """
        # Get current encoder readings
        left_ticks = msg.data[0]
        right_ticks = msg.data[1]
        
        # Initialize reference ticks on first callback
        if self.first_callback:
            self.last_left_ticks = left_ticks
            self.last_right_ticks = right_ticks
            self.first_callback = False
            return
        
        # Calculate true tick deltas considering overflow
        delta_left = self.handle_overflow(left_ticks, self.last_left_ticks)
        delta_right = self.handle_overflow(right_ticks, self.last_right_ticks)
        
        # Accumulate total ticks (useful for debugging and distance calculations)
        self.total_left_ticks += delta_left
        self.total_right_ticks += delta_right
        
        # Update last tick values for next callback
        self.last_left_ticks = left_ticks
        self.last_right_ticks = right_ticks
        
        # Convert ticks to distance traveled by each wheel
        meters_per_tick = (2 * math.pi * self.WHEEL_RADIUS) / self.TICKS_PER_REVOLUTION
        delta_left_distance = delta_left * meters_per_tick
        delta_right_distance = delta_right * meters_per_tick
        
        # Calculate robot motion using differential drive kinematics
        delta_distance = (delta_right_distance + delta_left_distance) / 2  # Average forward distance
        delta_theta = (delta_right_distance - delta_left_distance) / self.WHEEL_BASE  # Change in orientation
        
        # Update robot pose
        self.theta += delta_theta
        # Use average orientation for better position approximation
        avg_theta = self.theta - (delta_theta / 2)
        self.x += delta_distance * math.cos(avg_theta)
        self.y += delta_distance * math.sin(avg_theta)
        
        # Log total accumulated ticks (useful for debugging)
        rospy.logdebug(f"Total ticks - Left: {self.total_left_ticks}, Right: {self.total_right_ticks}")
        rospy.logdebug(f"Position - X: {self.x:.3f}, Y: {self.y:.3f}, Theta: {math.degrees(self.theta):.2f}°")
        
    def publish_odometry(self, event):
        """
        Publish odometry message and broadcast transform
        
        Args:
            event: Timer event (not used)
        """
        # Create and fill odometry message
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "odom"      # Parent frame
        odom.child_frame_id = "base_link"  # Robot frame
        
        # Set position
        odom.pose.pose.position = Point(self.x, self.y, 0.0)
        
        # Convert euler angle to quaternion and set orientation
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, self.theta)
        odom.pose.pose.orientation = Quaternion(*q)
        
        # Publish odometry message
        self.odom_pub.publish(odom)
        
        # Create and broadcast transform
        t = TransformStamped()
        t.header.stamp = odom.header.stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = odom.pose.pose.orientation
        
        self.br.sendTransform(t)

if __name__ == '__main__':
    try:
        calculator = OdometryCalculator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass