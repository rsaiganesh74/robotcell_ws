#!/usr/bin/env python3

from flask import Flask,request,jsonify
import requests

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from interfaces.srv import BarcodeNumberScanned
from interfaces.action import PickRequest

import threading

global robot_http_server

class RobotHTTPServer(Node):
    def __init__(self):
        super().__init__('robot_http_server')

        self.reentrant_callback = ReentrantCallbackGroup()

        self.pick_client = ActionClient(self,PickRequest,'pick_request',callback_group=self.reentrant_callback)
        self.barcode_service_client= self.create_client(BarcodeNumberScanned,'barcode_request',callback_group=self.reentrant_callback)

        self.pick_request = PickRequest.Goal()
        self.barcode_scan_request = BarcodeNumberScanned.Request()

        self.barcode_number_received = 0

        self.pick_status=""
        self.pick_status_reason=""

        while not self.barcode_service_client.wait_for_service(timeout_sec=10):
            self.get_logger().info('Waiting for barcode scanner service..')

    def execute_barcode_request(self,request):
        future = self.barcode_service_client.call_async(request)
        rclpy.spin_until_future_complete(self,future)
        self.barcode_number_received = future.result().barcode_num_scanned

        if self.barcode_number_received!=0:
            self.get_logger().info('Barcode Received. Proceeding ahead with execution..')
            return True
        else:
            return False
        
    def send_goal(self, num_of_items):
        self.pick_request.number_of_items = num_of_items
        self.pick_client.wait_for_server()
        self._send_goal_future = self.pick_client.send_goal_async(self.pick_request)
        rclpy.spin_until_future_complete(self, self._send_goal_future)
        goal_handle = self._send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal was rejected by action server.')
            return 
        
        self.get_logger().info('Goal accepted')

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        return result

    
app = Flask(__name__)

task_completion_url = "http://localhost:8081/confirmpick"

@app.route('/pick', methods=['POST'])
def pick():
    global robot_http_server

    data = request.get_json()
    pick_id = data.get("pick_id")
    quantity = data.get("quantity")
    robot_http_server.pick_request.number_of_items = quantity
    
    robot_http_server.get_logger().info(f"Received pick request : {pick_id} and for {quantity} items.")

    #1 get barcode
    if not robot_http_server.execute_barcode_request(robot_http_server.barcode_scan_request):
            confirm_data = {
                "pickId": pick_id,
                "pickSuccessful": "false",
                "errorMessage": "Barcode Scanner faulty",
                "itemBarcode": 0
                }
            try:
                response = requests.post(task_completion_url,json=confirm_data)
                response.raise_for_status()
                robot_http_server.get_logger().info(f"Confirmation sent to WMS, received: {response.json()}")
            except Exception as e:
                robot_http_server.get_logger().info("Error sending confirmation : ",e)

            return jsonify({"status":"Pick request Failed"}),400
    
    result = robot_http_server.send_goal(quantity)

    robot_http_server.pick_status = "true" if result.status else "false"
    robot_http_server.pick_status_reason = result.reason
    
    confirm_data = {
                "pickId": pick_id,
                "pickSuccessful": robot_http_server.pick_status,
                "errorMessage": robot_http_server.pick_status_reason,
                "itemBarcode": robot_http_server.barcode_number_received
    }

    try:
        response = requests.post(task_completion_url,json=confirm_data)
        response.raise_for_status()
        robot_http_server.get_logger().info(f"Confirmation sent to WMS, received: {response.json()['status']}")
    except Exception as e:
        robot_http_server.get_logger().info("Error sending confirmation : ",e)

    return jsonify({"status":"Pick request Succeeded"}),200

def main():
    global robot_http_server

    rclpy.init()

    robot_http_server = RobotHTTPServer()
    
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(robot_http_server)

    # 4) Spin in a background thread
    def ros_spin():
        try:
            executor.spin()
        except KeyboardInterrupt:
            pass
    try:
        ros_thread = threading.Thread(target=ros_spin, daemon=True) #running the ros spin in a different thread so as to not block flask
        ros_thread.start()

        # reloader is false so we don't spawn multiple processes.
        app.run(port=8080, debug=True, use_reloader=False, threaded=True)
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()