#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int8
from std_msgs.msg import Bool

class StackLight(Node):
    def __init__(self):
        super().__init__('stack_light_node')
        
        self.publish_interval = 0.01 #rate at which the publisher published the stack light status

        self.pub = self.create_publisher(Int8,'stack_light_status',10) #keeping 10 as depth so that subscriber has access to 10 last numbers in case of delay

        self.create_subscription(Bool,'door_status',self.cb_door_status,10)

        self.create_subscription(Bool,'estop_status',self.cb_estop_status,10)

        self.timer = self.create_timer(self.publish_interval,self.cb_publish)

        self.stack_light_status_msg = Int8() #defining the msg here itself to avoid redefinition in publisher callback constantly
        
        self.door_status = True #initially door closed

        self.estop_status = False #initiall estop disengaged

        self.stack_light_state = 0 #assuming door is closed and estop is not pressed initially

    def cb_publish(self):
        if self.estop_status:
            self.stack_light_state = -1
            self.stack_light_status_msg.data = self.stack_light_state
        elif not self.door_status:
            self.stack_light_state = 1
            self.stack_light_status_msg.data = self.stack_light_state
        else:
            self.stack_light_state = 0
            self.stack_light_status_msg.data = self.stack_light_state            
        self.pub.publish(self.stack_light_status_msg)

    def cb_door_status(self,msg_status):
        if msg_status.data == self.door_status:
            pass
        else:
            self.door_status = msg_status.data

    def cb_estop_status(self,msg_status):
        if msg_status.data == self.estop_status:
            pass
        else:
            self.estop_status = msg_status.data
            
def main():
    rclpy.init()

    stack_light = StackLight()

    try:
        rclpy.spin(stack_light)
    except KeyboardInterrupt:
        pass

if __name__=="__main__":
    main()