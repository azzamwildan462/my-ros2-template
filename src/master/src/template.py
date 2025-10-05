#!/usr/bin/python3

import rclpy
from rclpy.node import Node 

from loguru import logger

class Template(Node):
    def __init__(self):
        super().__init__('template_node')
        self.get_logger().info("Template Node has been started.")

        # Logger
        # ------
        logger.remove()
        logger.add(
            sys.stdout,
            colorize=True,
            format="<green>{time:HH:mm:ss.SSS}</green> | <level>{level:^6}</level> | <cyan>{function}</cyan>:<cyan>{line}</cyan> - {message}",
            enqueue=True,
        )

        # Example of a timer that calls a callback every second
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info("Template Node is running...")

def main(args=None):
    rclpy.init(args=args)

    node_template = Template()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node_template)
    executor.spin()
    
if __name__ == '__main__':
    main(sys.argv)