#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import time

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')

        self.reentrant_callback_group = ReentrantCallbackGroup()

        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10,
            callback_group=self.reentrant_callback_group)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        time.sleep(2.0)
        

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    # Use MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(minimal_subscriber)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        minimal_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()