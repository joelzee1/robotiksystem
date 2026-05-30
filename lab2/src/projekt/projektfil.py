#! /usr/bin/env python3
import numpy as np
from geometry_msgs.msg import TwistStamped
from PIL import Image
import math
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from tf_transformations import euler_from_quaternion
import heapq


class ProjektFil(Node):

    def __init__(self):
        super().__init__("projektfil")

        
        self.pose = Pose()
        self.yaw = 0.0
        self.has_odom = False

       
        self.path = None
        self.waypoints = []
        self.current_waypoint_index = 0

      
        self.lookahead_distance = 0.3
        self.forward_velocity = 0.15

       
        self.goal_x = 2.0
        self.goal_y = 3.0

        
        self.load_map()

       
        self.odom_sub = self.create_subscription(
            Odometry, "odom", self.get_odom_callback, qos_profile_sensor_data
        )

        qos = QoSProfile(depth=10)
        self.cmd_vel_pub = self.create_publisher(TwistStamped, "/cmd_vel", qos)

        self.timer = self.create_timer(0.1, self.timer_callback)

   
    def get_odom_callback(self, msg):
        self.pose = msg.pose.pose

        q = self.pose.orientation
        (_, _, self.yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])

        self.has_odom = True

   
    def load_map(self):
        img = Image.open("map.pgm")
        grid = np.array(img)

      
        self.grid = np.where(grid > 50, 1, 0)

        with open("map.yaml", "r") as f:
            map_info = yaml.safe_load(f)

        self.resolution = map_info["resolution"]
        self.origin_x = map_info["origin"][0]
        self.origin_y = map_info["origin"][1]

    
    def timer_callback(self):

        if not self.has_odom:
            return

      
        if self.path is None:
            self.plan_path()
            if self.path is None:
                return

            self.waypoints = [
                grid_to_world(r, c,
                              self.resolution,
                              self.origin_x,
                              self.origin_y)
                for r, c in self.path
            ]

        if len(self.waypoints) == 0:
            return

        rx = self.pose.position.x
        ry = self.pose.position.y

        target = self.get_lookahead_point(rx, ry)
        if target is None:
            return

        v, omega = self.pure_pursuit(rx, ry, self.yaw, target)

        twist = TwistStamped()
        twist.twist.linear.x = v
        twist.twist.angular.z = omega

        if self.goal_reached():
            twist.twist.linear.x = 0.0
            twist.twist.angular.z = 0.0

        self.cmd_vel_pub.publish(twist)

    
    def plan_path(self):

        start = world_to_grid(
            self.pose.position.x,
            self.pose.position.y,
            self.resolution,
            self.origin_x,
            self.origin_y
        )

        goal = world_to_grid(
            self.goal_x,
            self.goal_y,
            self.resolution,
            self.origin_x,
            self.origin_y
        )

        self.path = self.a_star(start, goal)

    def a_star(self, start, goal):

        open_set = []
        heapq.heappush(open_set, (0, start))

        came_from = {}
        g_score = {start: 0}

        neighbors = [
            (-1, 0), (1, 0),
            (0, -1), (0, 1),
            (-1, -1), (-1, 1),
            (1, -1), (1, 1)
        ]

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal:
                return reconstruct_path(came_from, current)

            for dr, dc in neighbors:

                nr = current[0] + dr
                nc = current[1] + dc

                # bounds
                if nr < 0 or nr >= self.grid.shape[0]:
                    continue
                if nc < 0 or nc >= self.grid.shape[1]:
                    continue

                # obstacle check
                if self.grid[nr, nc] == 1:
                    continue

                neighbor = (nr, nc)

                cost = 1.414 if dr != 0 and dc != 0 else 1.0
                tentative_g = g_score[current] + cost

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    g_score[neighbor] = tentative_g
                    came_from[neighbor] = current
                    priority = tentative_g + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (priority, neighbor))

        return None

    def pure_pursuit(self, rx, ry, yaw, target):

        tx, ty = target

        dx = tx - rx
        dy = ty - ry

        local_x = math.cos(-yaw) * dx - math.sin(-yaw) * dy
        local_y = math.sin(-yaw) * dx + math.cos(-yaw) * dy

        if abs(local_x) < 1e-6:
            return 0.0, 0.0

        curvature = (2 * local_y) / (local_x**2 + local_y**2)

        v = self.forward_velocity
        omega = v * curvature

        return v, omega

    
    def get_lookahead_point(self, rx, ry):

        for i in range(self.current_waypoint_index, len(self.waypoints)):

            x, y = self.waypoints[i]
            dist = math.hypot(x - rx, y - ry)

            if dist > self.lookahead_distance:
                self.current_waypoint_index = i
                return (x, y)

        return self.waypoints[-1]

   
    def goal_reached(self):

        if not self.waypoints:
            return False

        rx = self.pose.position.x
        ry = self.pose.position.y

        gx, gy = self.waypoints[-1]

        return math.hypot(gx - rx, gy - ry) < 0.2




def world_to_grid(x, y, resolution, origin_x, origin_y):
    col = int((x - origin_x) / resolution)
    row = int((y - origin_y) / resolution)
    return row, col


def grid_to_world(row, col, resolution, origin_x, origin_y):
    x = col * resolution + origin_x
    y = row * resolution + origin_y
    return x, y


def heuristic(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def reconstruct_path(came_from, current):
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path



def main(args=None):
    rclpy.init(args=args)
    node = ProjektFil()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()