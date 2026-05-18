#! /usr/bin/env python3
from geometry_msgs.msg import TwistStamped
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
 
    def __init__(self):
        super().__init__("obstacle_detection")
        
     
        self.declare_parameter("stop_distance", 0.25) 
        self.stop_distance = (self.get_parameter("stop_distance").get_parameter_value().double_value)
        self.get_logger().info(f"Using stop_distance: {self.stop_distance}m")
        self.pose = Pose()

        self.odom_sub = self.create_subscription(Odometry, "odom", self.get_odom_callback, qos_profile=qos_profile_sensor_data)
        self.scan_ranges = []
        self.has_scan_received = False
        self.avoiding = False

        self.tele_twist = TwistStamped()
        self.tele_twist.twist.linear.x = 0.0
        self.tele_twist.twist.angular.z = 0.0
        self.pose.position.x = 0.0
        self.pose.position.y = 0.0
        
        self.yaw = 0.0
        self.has_odom = False

        self.obstacle_distance = float("inf")
        self.obstacle_angle = 0.0
        self.x_obstacle = 0.0
        self.y_obstacle = 0.0
        
    
        qos = QoSProfile(depth=10)

        self.cmd_vel_pub = self.create_publisher(TwistStamped, "/cmd_vel", qos)
        
        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, qos_profile=qos_profile_sensor_data
        )

        self.cmd_vel_raw_sub = self.create_subscription(
            TwistStamped, "cmd_vel_raw", self.cmd_vel_raw_callback, 
            qos_profile=qos_profile_sensor_data
        )

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

            if not math.isfinite(r):
                continue

            angle = msg.angle_min + i * msg.angle_increment

            angle = math.atan2(math.sin(angle), math.cos(angle))


            valid_ranges.append((i, r, angle))

            

        if not valid_ranges:
            return

        filtered = [(r, angle) for (_, r, angle) in valid_ranges if r > 0.05]

        if not filtered:
            return

        d, angle = min(filtered, key=lambda x: x[0])

        self.obstacle_distance = d
        self.obstacle_angle = angle
        

        self.has_scan_received = True
            
    
    def cmd_vel_raw_callback(self, msg):
        self.tele_twist = msg

    def timer_callback(self):
            self.detect_obstacle()
        
    def detect_obstacle(self):
     
        
        xgoal = 1.0
        ygoal = -4.3
        Kp = 0.7
        Ko = 2.0
        goal_diff = 0.2
        
        twist= TwistStamped()

        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = "base_link"


        #twist.twist.linear.x = 0.1
        #twist.twist.angular.z = -0.0
        
        
        dx = xgoal - self.pose.position.x
        dy = ygoal - self.pose.position.y

        distance_to_goal = math.sqrt(dx**2 + dy**2)

        
        desired_angle = math.atan2(dy,dx)
        angular_difference = math.atan2(math.sin(desired_angle - self.yaw), math.cos(desired_angle - self.yaw))
 
        
        
        avoid_turn = self.obstacle_angle

        w_avoid = max(0.0, 1.0 - self.obstacle_distance / 0.4)
        w_gtg = 1.0 - w_avoid


        blended_turn = w_gtg * angular_difference + w_avoid * avoid_turn
        twist.twist.angular.z = blended_turn

        twist.twist.linear.x = 0.1 * w_gtg
        
        if distance_to_goal < goal_diff:

            twist.twist.linear.x = 0.0
            twist.twist.angular.z = 0.0
            self.get_logger().info("Goal reached!")
    

        #if not self.has_scan_received or self.obstacle_distance == float("inf"):
        #    twist.twist.linear.x = 0.1
        #    twist.twist.angular.z = 0.0
        #    self.cmd_vel_pub.publish(twist)
        #    return
       
        self.cmd_vel_pub.publish(twist)
   
        
        
        
       
    
    

    def destroy_node(self):
        self.get_logger().info("Shutting down, stopping robot...")
        stop_twist = TwistStamped() 
        stop_twist.header.stamp = self.get_clock().now().to_msg()
        stop_twist.header.frame_id = "base_link"
        self.cmd_vel_pub.publish(stop_twist)
        super().destroy_node() 


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
