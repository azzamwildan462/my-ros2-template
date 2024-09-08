#!/usr/bin/python3

import os
import sys
import json
from io import StringIO
import pandas as pd
from loguru import logger

import rclpy
from rclpy.node import Node

from flask import Flask, request, jsonify
from flask_cors import CORS

DEBUG_LEVEL = os.getenv("DEBUG_LEVEL", "INFO")
SERVER_PORT = os.getenv("DEBUG_LEVEL", "5000")

class Server(Node):
    def __init__(self):
        self.lock = Lock()

        super().__init__("server")

        # Init
        # ----
        self.routine_init()

    # ========================================================================================
    
    def routine_init(self):
        time.sleep(2.0)

        self.app = Flask(__name__)
        CORS(self.app)

        self.flask_router()

        # Flask
        flask_thread = threading.Thread(target=self.thread_flask)
        flask_thread.daemon = True
        flask_thread.start()

    # ========================================================================================
    # FLASK Controller and Model 

    def flask_ok(self):
        return jsonify({"status": 0, "message": "OK"})

    # ========================================================================================
    # FLASK Router

    def flask_router(self):
        @self.app.route("/", methods=["GET"])
        def flask_index():
            return self.flask_ok()

    def thread_flask(self):
        self.app.run(host="0.0.0.0", port=int(SERVER_PORT))

    # ========================================================================================


def main(args=None):
    rclpy.init(args=args)

    node_server = Server()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node_server)
    executor.spin()



if __name__ == "__main__":
    main(sys.argv)