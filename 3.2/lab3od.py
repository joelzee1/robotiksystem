#! /usr/bin/env python3
from geometry_msgs.msg import Twist,TwistStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
from rclpy.clock import Clock
from rclpy.clock_type import ClockType
from sensor_msgs.msg import LaserScan
import math

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from tf_transformations import euler_from_quaternion

class ObstacleDetection(Node):
    """
    Simple obstacle detection node that stops the robot when obstacles are too close.
    Uses a circular detection zone around the robot.
    
    TODO: Implement the detect_obstacle method to avoid obstacles!
    """
    def __init__(self):
        super().__init__("obstacle_detection")
        
        # Safety parameters - use ROS parameter
        self.declare_parameter("stop_distance", 0.25)  # Default if not specified
        self.stop_distance = (self.get_parameter("stop_distance").get_parameter_value().double_value)
        self.get_logger().info(f"Using stop_distance: {self.stop_distance}m")
        self.pose = Pose()
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.get_odom_callback, qos_profile=qos_profile_sensor_data)
        # Store received data
        self.scan_ranges = []
        self.has_scan_received = False
        self.avoiding = False
        # Default motion command (slow forward)
        self.tele_twist = TwistStamped()
        self.tele_twist.twist.linear.x = 0.5
        self.tele_twist.twist.angular.z = 0.0
        self.yaw = 0.0
        self.has_odom = False

        self.obstacle_distance = float("inf")
        self.x_obstacle = 0.0
        self.y_obstacle = 0.0
       
        # Set up quality of service
        qos = QoSProfile(depth=10)

        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(TwistStamped, "cmd_vel", qos)
        
        # Subscribe to laser scan data
        self.scan_sub = self.create_subscription(
            LaserScan, "scan", self.scan_callback, qos_profile=qos_profile_sensor_data
        )

        # Subscribe to teleop commands
        self.cmd_vel_raw_sub = self.create_subscription(
            TwistStamped, "cmd_vel_raw", self.cmd_vel_raw_callback, 
            qos_profile=qos_profile_sensor_data
        )

        # Set up timer for regular checking
        self.timer = self.create_timer(0.1, self.timer_callback,clock=Clock(clock_type=ClockType.STEADY_TIME))

    def get_odom_callback(self, msg):
        self.pose = msg.pose.pose
        
        oriList = [
            self.pose.orientation.x, 
            self.pose.orientation.y, 
            self.pose.orientation.z, 
            self.pose.orientation.w
            ]
        (_, _, self.yaw) = euler_from_quaternion(oriList)
        self.get_logger().info(f"Robot state  {self.pose.position.x, self.pose.position.y, self.yaw}")
        self.has_odom = True
    def scan_callback(self, msg):
        valid_ranges = []

        for i, r in enumerate(msg.ranges):

            # Ignore invalid readings
            if not math.isfinite(r):
                continue

            # Compute angle of this beam
            angle = msg.angle_min + i * msg.angle_increment

            # Normalize angle to [-pi, pi]
            angle = math.atan2(math.sin(angle), math.cos(angle))

            # Keep only front 180 degrees
            if -math.pi/2 <= angle <= math.pi/2:
                valid_ranges.append((i, r, angle))

        # No valid points
        if not valid_ranges:
            return

        # Find closest obstacle in selected region
        i, d, angle = min(valid_ranges, key=lambda x: x[1])
        self.obstacle_distance = d
        self.x_obstacle = d * math.cos(angle)
        self.y_obstacle = d * math.sin(angle)
        

        self.has_scan_received = True
            

    def cmd_vel_raw_callback(self, msg):
        """Store teleop commands when received"""
        self.tele_twist = msg

    def timer_callback(self):
        if not self.has_odom:
            return
        """Regular function to check for obstacles"""
        if self.has_scan_received:
            self.detect_obstacle()
        
    def detect_obstacle(self):
     
        
        xgoal = 3.0
        ygoal = 3.0
        Kp = 0.7
        Ko = 2.0
        goal_diff = 0.2
        
        twisted = TwistStamped()

        dx = xgoal - self.pose.position.x
        dy = ygoal - self.pose.position.y

        distance_to_goal = math.sqrt(dx**2 + dy**2)


        desired_angle = math.atan2(dy,dx)
        angular_difference = math.atan2(math.sin(desired_angle - self.yaw), math.cos(desired_angle - self.yaw))
 
        
        
        obstacle_angle = math.atan2(self.y_obstacle,self.x_obstacle)
        
        
    
        avoid_turn = -obstacle_angle
        w_avoid = max(0.0, 1.0 - self.obstacle_distance / 0.3)
        w_gtg = 1.0 - w_avoid
        blended_turn = w_gtg * angular_difference + w_avoid * avoid_turn
        twisted.twist.angular.z = blended_turn

        twisted.twist.linear.x = 0.5 * w_gtg 
        
        if distance_to_goal < goal_diff:

            twisted.twist.linear.x = 0.5
            twisted.twist.angular.z = 0.0
            self.get_logger().info("Goal reached!")
    



        self.cmd_vel_pub.publish(twisted)
        
        
        
       
    
    

    def destroy_node(self):
        """Publish zero velocity when node is destroyed"""
        self.get_logger().info("Shutting down, stopping robot...")
        stop_twist = TwistStamped() # Default Twist has all zeros
        self.cmd_vel_pub.publish(stop_twist)
        super().destroy_node() # Call the parent class's destroy_node


def main(args=None):
    rclpy.init(args=args)
    obstacle_detection = ObstacleDetection()
    try:
        rclpy.spin(obstacle_detection)
    except KeyboardInterrupt:
        obstacle_detection.get_logger().info('KeyboardInterrupt caught, allowing rclpy to shutdown.')
    finally:
        obstacle_detection.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
