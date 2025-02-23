#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from std_srvs.srv import SetBool

class Estop(Node):
    def __init__(self):
        super().__init__('estop_node')
        
        self.publish_interval = 0.01 #rate at which the publisher published the estop status

        self.pub = self.create_publisher(Bool,'estop_status',10) #keeping 10 as depth so that subscriber has access to 10 last numbers in case of delay

        self.create_service(SetBool,'estop_status_set',self.cb_estop_request)

        self.timer = self.create_timer(self.publish_interval,self.cb_publish)

        self.estop_status_msg = Bool() #defining the msg here itself to avoid redefinition in publisher callback constantly

        self.estop_state = False #assuming estop is not pressed initially

    def cb_publish(self):
        self.estop_status_msg.data = self.estop_state
        self.pub.publish(self.estop_status_msg)

    def cb_estop_request(self,request,response):
        if self.estop_state:
            if request.data:
                response.success = False
                response.message = "Failed to execute request. Emergency Stop button is already pressed."
            else:
                response.success = True
                response.message = "Request successfully executed. Emergency Stop button is disengaged now."
        else:
            if request.data:
                response.success = True
                response.message = "Request successfully executed. Emergency Stop button is now pressed."
            else:
                response.success = False
                response.message = "Failed to execute request. Emergency Stop button is already disengaged."
        if response.success:
            self.estop_state = not self.estop_state

        self.get_logger().info(f'{response.message}')
        return response

def main():
    rclpy.init()

    estop = Estop()

    try:
        rclpy.spin(estop)
    except KeyboardInterrupt:
        pass

if __name__=="__main__":
    main()