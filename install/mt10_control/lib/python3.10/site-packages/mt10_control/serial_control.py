import rclpy
from rclpy.node import Node 
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import serial

class RoverBase(Node):

    def __init__(self):
        super().__init__("rover_base")
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('port', '/dev/ttyACM0')

        cmd_vel = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        port = self.get_parameter('port').get_parameter_value().string_value

        self.rover = self.create_subscription(Twist, cmd_vel, self.listener_callback, 10)
        self.rover_controller = serial.Serial(port, baudrate= 9600)

        print(f"Rover base initialized with port, {port}, and subscription to {cmd_vel}")

    def listener_callback(self, msg):
        lin_vel = msg.linear.x
        ang_vel = msg.angular.z
        print(type(lin_vel))
        self.get_logger().info("I recieved: '%s'" % msg)
        data = f"{lin_vel} {ang_vel}"
        self.ser.write(data.encode("UTF-8"))

def main(args=None):
    rclpy.init(args=args)

    rover = RoverBase()

    rclpy.spin(rover)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
