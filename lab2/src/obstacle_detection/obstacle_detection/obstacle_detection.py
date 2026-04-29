#! /usr/bin/env python3
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
from sensor_msgs.msg import LaserScan
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from tf_transformations import euler_from_quaternion
from time import sleep

class ObstacleDetection(Node):
    """
    Simple obstacle detection node that stops the robot when obstacles are too close.
    Uses a circular detection zone around the robot.
    
    TODO: Implement the detect_obstacle method to avoid obstacles!
    """
    def __init__(self):
        super().__init__("obstacle_detection")
        
        # Safety parameters - use ROS parameter
        self.declare_parameter("stop_distance", 0.35)  # Default if not specified
        self.stop_distance = self.get_parameter("stop_distance").get_parameter_value().double_value
        self.get_logger().info(f"Using stop_distance: {self.stop_distance}m")
        self.pose = Pose()
        self.odom_sub = self.create_subscription(Odometry, "odom", self.get_odom_callback, qos_profile=qos_profile_sensor_data)
        # Store received data
        self.scan_ranges = []
        self.has_scan_received = False
        self.avoiding = False
        # Default motion command (slow forward)
        self.tele_twist = Twist()
        self.tele_twist.linear.x = 0.0
        self.tele_twist.angular.z = 0.0
       
        # Set up quality of service
        qos = QoSProfile(depth=10)

        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", qos)
        
        # Subscribe to laser scan data
        self.scan_sub = self.create_subscription(
            LaserScan, "scan", self.scan_callback, qos_profile=qos_profile_sensor_data
        )

        # Subscribe to teleop commands
        self.cmd_vel_raw_sub = self.create_subscription(
            Twist, "cmd_vel_raw", self.cmd_vel_raw_callback, 
            qos_profile=qos_profile_sensor_data
        )

        # Set up timer for regular checking
        self.timer = self.create_timer(0.1, self.timer_callback)

    def get_odom_callback(self, msg):
        self.pose = msg.pose.pose
        
        oriList = [self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w]
        (roll, pitch, self.yaw) = euler_from_quaternion(oriList)
        self.get_logger().info(f"Robot state  {self.pose.position.x, self.pose.position.y, self.yaw}")

    def scan_callback(self, msg):
        """Store laser scan data when received"""
        self.scan_ranges = msg.ranges
            # Filtrera bort ogiltiga värden (inf, nan)
        valid_ranges = [(i, r) for i, r in enumerate(msg.ranges) if math.isfinite(r)]

        # Närmaste hinder
        i, d = min(valid_ranges, key=lambda x: x[1])
        angle = msg.angle_min + i * msg.angle_increment

        # OBS! Vissa Lidar-sensorer rapporterar vinklar 0 till 2*pi. 
        # Normalisera till [-pi, pi] annars blir roboten blind på sin högra sida!
        angle = math.atan2(math.sin(angle), math.cos(angle))

        # Hindrets position relativt roboten
        self.x_obstacle = d * math.cos(angle)
        self.y_obstacle = d * math.sin(angle)
        self.obstacle_distance = d
        self.has_scan_received = True
        

    def cmd_vel_raw_callback(self, msg):
        """Store teleop commands when received"""
        self.tele_twist = msg

    def timer_callback(self):
        """Regular function to check for obstacles"""
        if self.has_scan_received:
            self.detect_obstacle()

    def detect_obstacle(self):


        xgoal = 3.0
        ygoal = 3.0
        Kp = 4
        goal_diff = 0.2
        desired_angle = math.atan2(ygoal - self.pose.position.y, xgoal - self.pose.position.x)
        
        desired_angle = math.atan2(ygoal - self.pose.position.y, xgoal - self.pose.position.x)
        angular_difference = math.atan2(math.sin(desired_angle - self.yaw), math.cos(desired_angle - self.yaw))
        self.tele_twist.angular.z = Kp * angular_difference
        if(goal_diff > math.sqrt((xgoal-self.pose.position.x)**2 + (ygoal-self.pose.position.y)**2)):
            self.tele_twist.linear.x = 0.0
        

        """
        TODO: Implement obstacle detection and avoidance!
        
        MAIN TASK:
        - Detect if any obstacle is too close to the robot (closer than self.stop_distance)
        - Turn if obstacle is close
        
        UNDERSTANDING LASER SCAN DATA:
        - self.scan_ranges contains distances to obstacles in meters
        - Each value represents distance at a different angle around the robot
        - Values less than self.stop_distance indicate a close obstacle
        
        UNDERSTANDING POSE (Pose message)
        - self.pose.position.x: x position of the robot
        - self.pose.position.y: y position of the robot
        - yaw: heading of the robot, converted from quaternions for your convinence (in radians)

        CREATE CONTROL SIGNAL FOR ANGULAR VELOCITY
        - Compare angle to goal or obstacle with the current angle of the robot, i.e
        - e_theta = (gtg-yaw)
        - make sure it wraps between pi and -pi
        - e_theta = atan2(sin(e_theta), cos(e_theta))
        - twist.angular.z = P * e_theta
        - Choose P
        
        CONTROLLING THE ROBOT (Twist message):
        - twist.linear.x: Forward/backward (positive = forward, negative = backward)
        - twist.angular.z: Rotation (positive = left, negative = right)
        - To stop: set twist.linear.x = 0.0 (you can keep angular.z to allow turning)
        """
        # Filter out invalid readings (very small values, infinity, or NaN)
        valid_ranges = [r for r in self.scan_ranges if not math.isinf(r) and not math.isnan(r) and r > 0.01]
        
        # If no valid readings, assume no obstacles
        if not valid_ranges:
            self.cmd_vel_pub.publish(self.tele_twist)
            self.avoiding = False
            return
            
        # Find the closest obstacle in any direction (full 360° scan)
        
    
        
        if(self.obstacle_distance > 0.25):
            self.avoiding = False
        
        obstacle_angle = math.atan2(self.y_obstacle, self.x_obstacle)
        if(self.obstacle_distance <=  0.25):
            angle_diff = math.atan2(math.sin(desired_angle - obstacle_angle), math.cos(desired_angle - obstacle_angle))
            self.tele_twist.linear.x = 0.0
            if not self.avoiding:
                self.avoiding =True
                if(angle_diff > 0):
                    self.side = obstacle_angle + ((math.pi)/2)
                else:
                    self.side = obstacle_angle - ((math.pi)/2)
        
                
            new_route=self.side    
            angular_difference_o = math.atan2(math.sin(new_route - self.yaw), math.cos(new_route- self.yaw))
            self.tele_twist.angular.z =Kp * angular_difference_o
            if(angular_difference_o < 0.5 and angular_difference_o > -0.5):
                self.tele_twist.linear.x = 0.05
        
            
        else:
            if(not self.avoiding):

                desired_angle = math.atan2(ygoal - self.pose.position.y, xgoal - self.pose.position.x)
                angular_difference = math.atan2(math.sin(desired_angle - self.yaw), math.cos(desired_angle - self.yaw))
                self.tele_twist.angular.z = Kp * angular_difference
                self.tele_twist.linear.x = min(0.2, Kp * math.sqrt((xgoal-self.pose.position.x)**2 + (ygoal-self.pose.position.y)**2))
            if(goal_diff > math.sqrt((xgoal-self.pose.position.x)**2 + (ygoal-self.pose.position.y)**2)):
                self.tele_twist.linear.x = 0.0
        # TODO: Implement your obstacle detection logic here!
        # Remember to use obstacle_distance and self.stop_distance in your implementation.
        # Remember to find the angle of the closest obstacle
        

        # For now, just use the teleop command (unsafe - replace with your code)
        twist = self.tele_twist

        # Publish the velocity command
        self.cmd_vel_pub.publish(twist)

    def destroy_node(self):
        """Publish zero velocity when node is destroyed"""
        self.get_logger().info("Shutting down, stopping robot...")
        stop_twist = Twist() # Default Twist has all zeros
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
