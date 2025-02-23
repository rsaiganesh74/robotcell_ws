#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from interfaces.srv import BarcodeNumberScanned
from interfaces.action import PickRequest

from std_msgs.msg import Bool

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

class RobotServer(Node):
    def __init__(self):
        super().__init__('robot_server')

        self.reentrant_callback_group = ReentrantCallbackGroup()

        self.create_subscription(Bool,'door_status',self.cb_doorhandle,10,callback_group=self.reentrant_callback_group)
        self.create_subscription(Bool,'estop_status',self.cb_estop,10,callback_group=self.reentrant_callback_group)

        self.robot_server = ActionServer(self,PickRequest,'pick_request',self.cb_execute_request,callback_group=self.reentrant_callback_group)

        self.total_wait_duration = 0

        self.door = True #door closed assumption initially

        self.estop = False #estop disengaged initially assumption

        self.result = PickRequest.Result()


    def cb_execute_request(self,goal_handle):
        self.get_logger().info('Goal Request Recieved. Started execution..')

        self.total_wait_duration = goal_handle.request.number_of_items * 5 #5second runtime for each item

        start_time = self.get_clock().now().nanoseconds / 1e9

        while True:

            if self.estop:
                self.result.status = False
                self.result.reason = "estop"
                goal_handle.abort()
                self.get_logger().error(f"Task failed due to estop being pressed.")
                return self.result
            
            if not self.door:
                self.result = PickRequest.Result()
                self.result.status = False
                self.result.reason = "door"
                goal_handle.abort()
                self.get_logger().error(f"Task failed due to door being opened.")
                return self.result
            now = self.get_clock().now().nanoseconds / 1e9
            elapsed_time = now-start_time

            if elapsed_time >= self.total_wait_duration:
                self.result = PickRequest.Result()
                self.result.status = True
                self.result.reason = "null"
                goal_handle.succeed()
                self.get_logger().info(f"Task succeeded.")
                return self.result
    
    def cb_doorhandle(self,msg):
        if not msg.data==self.door:
            self.door = msg.data

    def cb_estop(self,msg):
        if not msg.data==self.estop:
            self.estop = msg.data

def main():
    rclpy.init()

    robot_server = RobotServer()

    executor = MultiThreadedExecutor()
    executor.add_node(robot_server)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

if __name__=="__main__":
    main()