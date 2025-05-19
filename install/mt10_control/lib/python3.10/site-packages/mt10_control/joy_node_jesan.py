#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

#we are using twist messege
from geometry_msgs.msg import Twist 

# we are   importing this for sending data to arduino
import serial
import time

# we are selecting arduino port
port="/dev/ttyACM1"
arduinoData=serial.Serial(port,115200)
time.sleep(1)
##############################
class Subscriber(Node):
    def __init__(self):
        super().__init__("joyCommandSubscriber")
        print("started getting data")
        self.create_subscription(Twist,"/turtle1/cmd_vel",self.getVelocityCallback,1)



    def getVelocityCallback(self,velocityMsg :Twist):
        # cmd=str(velocityMsg.linear.x)+str(velocityMsg.angular.z)
        # cmd+="$"

        cmd=int(velocityMsg.linear.x+1)
        # print("linear Speed: " + str(velocityMsg.linear.x))
        # print("Turn Speed: " +str(velocityMsg.angular.z))
        print(cmd)
        # arduinoData.write(cmd.encode())
        arduinoData.write(cmd.to_bytes(1,'little'))
        
def main(args=None):
    rclpy.init(args=args)
    node=Subscriber()
    rclpy.spin(node)
    rclpy.shutdown()
