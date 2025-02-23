#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from std_srvs.srv import SetBool

class DoorHandle(Node):
    def __init__(self):
        super().__init__('door_handle')
        
        self.publish_interval = 0.1 #rate at which the publisher published the door handle status

        self.pub = self.create_publisher(Bool,'door_status',10) #keeping 10 as depth so that subscriber has access to 10 last numbers in case of delay

        self.create_service(SetBool,'door_status_set',self.cb_door_request)

        self.timer = self.create_timer(self.publish_interval,self.cb_publish)

        self.door_status_msg = Bool() #defining the msg here itself to avoid redefinition in publisher callback constantly

        self.door_state = True #assuming door is closed initially

    def cb_publish(self):
        door_status = Bool()
        door_status.data = self.door_state
        self.pub.publish(door_status)

    def cb_door_request(self,request,response):
        if self.door_state:
            if request.data:
                response.success = False
                response.message = "Failed to execute request. Door is already closed."
            else:
                response.success = True
                response.message = "Request successfully executed. Door is now open."
        else:
            if request.data:
                response.success = True
                response.message = "Request successfully executed. Door is now closed."
            else:
                response.success = False
                response.message = "Failed to execute request. Door is already open."
        if response.success:
            self.door_state = not self.door_state

        self.get_logger().info(f'{response.message}')
        return response

def main():
    rclpy.init()

    door_handle = DoorHandle()

    try:
        rclpy.spin(door_handle)
    except KeyboardInterrupt:
        pass

if __name__=="__main__":
    main()