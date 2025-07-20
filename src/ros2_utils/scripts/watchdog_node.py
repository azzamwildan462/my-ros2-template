#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from composition_interfaces.srv import LoadNode
from threading import Lock
from loguru import logger


class Watchdog(Node):
    def __init__(self):
        self.lock = Lock()
        super().__init__('watchdog_node')

        logger.remove()
        logger.add(
            sys.stdout,
            colorize=True,
            format="<green>{time:HH:mm:ss.SSS}</green> | <level>{level:^6}</level> | <cyan>{function}</cyan>:<cyan>{line}</cyan> - {message}",
            enqueue=True,
        )

        self.declare_parameter("container_name", "/main_container")
        self.declare_parameter("watched_nodes", rclpy.Parameter.Type.STRING_ARRAY)

        self.container_name = self.get_parameter("container_name").get_parameter_value().string_value
        raw_nodes = self.get_parameter("watched_nodes").get_parameter_value().string_array_value

        self.watched_nodes = []
        for item in raw_nodes:
            try:
                parsed = eval(item)  # safe in trusted context
                if all(k in parsed for k in ("name", "plugin", "package")):
                    self.watched_nodes.append(parsed)
                else:
                    logger.warning(f"Incomplete node spec: {item}")
            except Exception as e:
                logger.error(f"Failed to parse watched node: {item} -> {e}")

        logger.info(f"Monitoring container: {self.container_name}")
        logger.info(f"Nodes to watch: {[n['name'] for n in self.watched_nodes]}")

        self.create_timer(5.0, self.check_nodes)

    def check_nodes(self):
        with self.lock:
            current_nodes = [n.removeprefix('/') for n, _ in self.get_node_names_and_namespaces()]
            for node in self.watched_nodes:
                if node["name"] not in current_nodes:
                    logger.warning(f"Node '{node['name']}' is not running. Restarting...")
                    self.reload_node(node)

    def reload_node(self, node_info):
        client = self.create_client(LoadNode, f"{self.container_name}/load_node")
        if not client.wait_for_service(timeout_sec=2.0):
            logger.error(f"Service not available: {client.srv_name}")
            return

        req = LoadNode.Request()
        req.package_name = node_info["package"]
        req.plugin_name = node_info["plugin"]
        req.node_name = node_info["name"]

        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)

        if future.result() is not None:
            if future.result().success:
                logger.success(f"Successfully reloaded node: {node_info['name']}")
            else:
                logger.error(f"Failed to reload node '{node_info['name']}': {future.result().error_message}")
        else:
            logger.error(f"Timeout or unknown error when loading node: {node_info['name']}")


def main(args=None):
    rclpy.init(args=args)
    watchdog = Watchdog()
    executor = MultiThreadedExecutor()
    executor.add_node(watchdog)
    executor.spin()
    watchdog.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
