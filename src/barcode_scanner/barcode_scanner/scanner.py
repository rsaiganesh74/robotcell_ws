#!/usr/bin/env python3

import random

import rclpy
from rclpy.node import Node

from interfaces.msg import BarcodeNumberGenerated
from interfaces.srv import BarcodeNumberScanned

class BarcodeScanner(Node):
    def __init__(self):
        super().__init__('barcode_scanner')
        
        self.publish_interval = 0.5 #rate at which the publisher published the barcode number

        self.pub = self.create_publisher(BarcodeNumberGenerated,'barcode',10) #keepinh 10 as depth so that subscriber has access to 10 last numbers in case of delay

        self.create_service(BarcodeNumberScanned,'barcode_request',self.cb_barcode_request)

        self.timer = self.create_timer(self.publish_interval,self.cb_publish)

        self.barcode_msg = BarcodeNumberGenerated() #defining the msg here itself to avoid redefinition in publisher callback constantly

        self.last_scanned_barcode = 0


    def cb_publish(self):
        self.barcode_msg.barcode_number_generated = random.randint(10000,99999)
        self.last_scanned_barcode = self.barcode_msg.barcode_number_generated
        self.pub.publish(self.barcode_msg)

    def cb_barcode_request(self,request,response):
        request #using the variable to avoid unused variable warning
        response.barcode_num_scanned = self.last_scanned_barcode
        self.get_logger().info(f'Incoming request for barcode scanned. Sent: {response.barcode_num_scanned}')
        return response

def main():
    rclpy.init()

    barcode_scanner = BarcodeScanner()

    try:
        rclpy.spin(barcode_scanner)
    except KeyboardInterrupt:
        pass

if __name__=="__main__":
    main()