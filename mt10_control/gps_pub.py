import rclpy
from rclpy.node import Node 
from std_msgs.msg import String, Bool
from sbg_driver.msg import SbgGpsPos
from time import sleep

class GPSSender(Node):
    def __init__(self) -> None:
        super().__init__("gps_sender")
        self.gps = self.create_publisher(SbgGpsPos, "/coordinates", 10)
        self.check_autonomous = self.create_subscription(Bool, "/autonomous_status", self.checker_callback, 10)
        self.status = False

    def checker_callback(self, msg: Bool):
        self.status = msg.data

    def sender(self):
        if self.status == True:
            print("Autonomous navigation is currently active")
        else:
            target_lat = float(input("Enter Target Latitude >"))
            target_lon = float(input("Enter Target Longitude >"))

            nav_msg = SbgGpsPos()
            nav_msg.latitude = target_lat
            nav_msg.longitude = target_lon

            self.gps.publish(nav_msg)
        
        sleep(0.2)

def main():
    rclpy.init()
    gpssender = GPSSender()
    
    while rclpy.ok():
        gpssender.sender()

    gpssender.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()