


import rospy
import tf
from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion, Point
import math

class OdometryCalculator:
    def __init__(self):
        
        """Initialize the odometry calculator with robot parameters and ROS setup"""
        
        # Robot physical parameters - ADJUST THESE FOR YOUR ROBOT
        self.WR  = 0.0325    # Wheel radius in meters
        self.WS  = 0.295     # Wheels Separation in meters
        self.PPR = 1440      # Encoder pulse per revolution
        
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

    
        self.first_encoder_reading = True  # Flag for initializing first encoder reading

                
        # Initialize ROS node
        rospy.init_node('odometry_calculator')

        # Set up ROS publishers and subscribers
        self.odom_pub    = rospy.Publisher('odom', Odometry, queue_size=10) # publish the odometry
        self.encoder_sub = rospy.Subscriber('encoder_ticks', Int32MultiArray, self.encoder_callback) # subscribe to the encoder ticks
        self.tf_broadcaster = tf.TransformBroadcaster()
        

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

        # Update odometry
        self.update_odometry(delta_right_ticks, delta_left_ticks)


    def update_odometry(self, Nr, Nl, dt):

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

        # Create odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        # Set position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        # Set orientation (quaternion)
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.theta)
        odom_msg.pose.pose.orientation = Quaternion(*odom_quat)

        # Set velocity
        odom_msg.twist.twist.linear.x = v
        odom_msg.twist.twist.angular.z = omega

        # Publish odometry message
        self.odom_pub.publish(odom_msg)

        # Broadcast transform (for visualization in RViz)
        self.tf_broadcaster.sendTransform(
            (self.x, self.y, 0.0),
            odom_quat,
            current_time,
            "base_link",
            "odom"
        )



def main():
    odom_calc = OdometryCalculator()
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

        


        