#!/usr/bin/env python3


import rospy
import tf
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
        self.WR  = 0.113    # Wheel radius in meters
        self.WS  = 0.315     # Wheels Separation in meters
        self.PPR = 11440      # Encoder pulse per revolution
        
        # Constants for 32-bit integer overflow detection
        self.MAX_TICKS = 2**31 - 1        # Maximum value for 32-bit signed integer
        self.MIN_TICKS = -2**31           # Minimum value for 32-bit signed integer
        self.OVERFLOW_THRESHOLD = 2**30   # Threshold to detect overflow (half of max value)


        # Odometry state variables >>> initial position of the robot
        self.x = 0.0          # Robot's X position in meters
        self.y = 0.0          # Robot's Y position in meters
        self.theta = 0.0      # Robot's orientation in radians


        self.last_time = rospy.Time.now() # Time of last encoder reading
        

        # Variables for tick tracking and overflow handling
        self.prev_left_ticks = None    # Previous left encoder reading
        self.prev_right_ticks = None   # Previous right encoder reading

        self.total_left_ticks = 0
        self.total_left_ticks = 0

        self.first_encoder_reading = True  # Flag for initializing first encoder reading

                
        

        # Set up ROS publishers and subscribers
        self.odom_pub    = rospy.Publisher('odom', Odometry, queue_size=10) # publish the odometry
        self.encoder_sub = rospy.Subscriber('/encoder_topic', Int32MultiArray, self.encoder_callback) # subscribe to the encoder ticks
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
            # Check if overflow in positive or negative direction
            if delta < 0:  # Overflow in positive direction
                delta = (self.MAX_TICKS - last_tick) + (current_tick - self.MIN_TICKS) + 1
            else:  # Overflow in negative direction
                delta = (self.MIN_TICKS - last_tick) + (current_tick - self.MAX_TICKS) - 1
                
        return delta



    def encoder_callback(self, msg):
        """Process encoder ticks and update odometry"""
        # Initialize reference ticks on first callback
        if self.first_encoder_reading:
            self.prev_right_ticks = msg.data[0]
            self.prev_left_ticks = msg.data[1]
            self.first_encoder_reading = False
            return

        # Calculate true tick deltas considering overflow
        delta_right_ticks = self.handle_overflow(msg.data[0], self.prev_right_ticks)  # Nr
        delta_left_ticks = self.handle_overflow(msg.data[1], self.prev_left_ticks)    # Nl

        self.prev_right_ticks = msg.data[0]
        self.prev_left_ticks = msg.data[1]


        self.total_right_ticks =+ msg.data[0]
        self.total_left_ticks =+ msg.data[1]

        # log the encoder ticks
        rospy.loginfo("Right ticks: %d, Left ticks: %d", delta_right_ticks, delta_left_ticks)

        # Update odometry
        self.update_odometry(delta_right_ticks, delta_left_ticks)


    def update_odometry(self, Nr, Nl):

        # Get current time
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        self.last_time = current_time
        
        # Calculate angular displacement of each wheel (in radians)
        delta_theta_right = (2 * math.pi * Nr) / self.PPR # theta_r = (2*pi*Nr)/PPR
        delta_theta_left  = (2 * math.pi * Nl) / self.PPR  # theta_l = (2*pi*Nl)/PPR

        # Calculate wheel velocities (in radians per second)
        omega_left = delta_theta_left / dt
        omega_right = delta_theta_right / dt

        # Calculate linear and angular wheel velocity
        v = (self.WR/2) * ( omega_right + omega_left)  # v = WR * (theta_r + theta_l) / 2
        omega = (self.WR/self.WS)  * ( omega_right - omega_left)      # w = WR * (theta_r - theta_l) / WS

        # Update robot pose
        self.x += v * math.cos(self.theta)*dt
        self.y += v * math.sin(self.theta)*dt
        self.theta += omega*dt

        # Normalize theta to the range [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
    
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
        odom_calc = OdometryCalculator()
        
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

        


        