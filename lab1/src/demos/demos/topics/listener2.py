#! /usr/bin/env python3
# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import String


class Listener(Node):
    def __init__(self1):
        super().__init__("listener1")
        self1.sub = self1.create_subscription(
            String, "chatter1", self1.chatter_callback, 10
        )
    def __init__(self2):
        super().__init__("listener2")
        self2.sub = self2.create_subscription(
            String, "chatter2", self2.chatter_callback, 10
        
        )
    def chatter_callback(self1, msg):
        self1.get_logger().info("I heard: [%s]" % msg.data)

    def chatter_callback(self2, msg):
        self2.get_logger().info("I heard: [%s]" % msg.data)


def main(args=None):
    rclpy.init(args=args)

    node = Listener()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
