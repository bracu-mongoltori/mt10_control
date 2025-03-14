import rclpy
from rclpy.node import Node 
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from sbg_driver.msg import SbgEkfEuler
from sbg_driver.msg import SbgGpsPos
from time import sleep
from math import radians, degrees, sin, cos, asin, sqrt, atan2


class Autonomous(Node):
    def __init__(self, max_lin_vel=45.0, max_ang_vel=7.0, yaw_threshold=10, distance_threshold=1.5):
        super().__init__("autonomous")
        self.autonomous = self.create_subscription(SbgGpsPos, "/coordinates", self.moveTo, 10)
        self.rover = self.create_publisher(Twist, "/cmd_vel", 10)
        self.gps = self.create_subscription(SbgGpsPos, "/sbg/gps_pos", self.gps_callback, 10)
        self.orientation = self.create_subscription(SbgEkfEuler, "/sbg/ekf_euler", self.orientation_callback, 10)
        self.status_pub = self.create_publisher(String, "/status", 10)
        self.autonomous_status = self.create_publisher(Bool, "/autonomous_status", 10)
        self.status_timer = self.create_timer(0.5, self.status_stuff)

        self.autonomous_on = False

        self.my_lat = 0.0
        self.my_lon = 0.0
        self.my_yaw = 0.0

        self.yaw_threshold = yaw_threshold
        self.distance_threshold = distance_threshold

        self.forward = Twist()
        self.left = Twist()
        self.right = Twist()
        self.backward = Twist()
        self.stop = Twist()

        self.forward.linear.x = max_lin_vel
        self.left.angular.z = max_ang_vel
        self.right.angular.z = -max_ang_vel
        self.backward.linear.x = -max_lin_vel

        print("Node initialized. Begin.")

    def distance_from_gps(self, lat1, lat2, lon1, lon2):
        #radius of earth in kilometers
        r = 6371

        lon1 = radians(lon1)
        lon2 = radians(lon2)
        lat1 = radians(lat1)
        lat2 = radians(lat2)
        
        #Haversine formula
        dlon = lon2 - lon1
        dlat = lat2 - lat1
        a = sin(dlat / 2)**2 + cos(lat1) * cos(lat2) * sin(dlon / 2)**2
        c = 2 * asin(sqrt(a))

        return c * r * 1000

    def bearing(self, curr_lat, curr_lon, target_lat, target_lon): #Bearing to waypoint (degrees)
        target_lat, target_lon, curr_lat, curr_lon = map(radians, [target_lat, target_lon, curr_lat, curr_lon])
        d_lon = target_lon - curr_lon
        return degrees(atan2(sin(d_lon) * cos(target_lat), cos(curr_lat) * sin(target_lat) - (sin(curr_lat) * cos(target_lat) * cos(d_lon))))

    def gps_callback(self, msg: SbgGpsPos):
        self.my_lat = msg.latitude
        self.my_lon = msg.longitude

    def orientation_callback(self, msg: SbgEkfEuler):
        self.my_yaw = msg.angle.z
        print(self.my_yaw)

    def status_stuff(self, msg=None):
        pubbed = String()
        bool_pub = Bool()
        if self.autonomous_on == False:
            bool_msg = False
            msg = "Rover autonomous navigation not active"
        else:
            bool_msg = True
        pubbed.data = msg
        bool_pub.data = bool_msg
        self.status_pub.publish(pubbed)
        print(msg)
        self.autonomous_status.publish(bool_pub)


    def moveTo(self, msg : SbgGpsPos):
        self.autonomous_on = True 

        target_lat = msg.latitude
        target_lon = msg.longitude

        print("Beginning Navigation")
        print(f"Current distance to target {self.distance_from_gps(self.my_lat, target_lat, self.my_lon, target_lon)}")

        while self.distance_from_gps(self.my_lat, target_lat, self.my_lon, target_lon) > self.distance_threshold:
            self.rover.publish(self.stop) # Needed for Real Rover

            # distance = haversine_distance(lat, lon, target_lat, target_lon)
            target_yaw = self.bearing(self.my_lat, self.my_lon, target_lat, target_lon)

            upper_limit = target_yaw + self.yaw_threshold
            lower_limit = target_yaw - self.yaw_threshold

            d_yaw = abs(target_yaw - self.my_yaw)

            self.status_stuff("GPS: ")
            self.status_stuff(f"lat: {self.my_lat}, lon: {self.my_lon}")
            self.status_stuff(f"Current distance to target: {self.distance_from_gps(self.my_lat, target_lat, self.my_lon, target_lon)}")
            self.status_stuff(f"Current yaw: {self.my_yaw}, target yaw: {target_yaw}")
            self.status_stuff(f"Angle left: {abs(target_yaw - self.my_yaw)}")
            self.status_stuff("")

            if d_yaw > 180:
                d_yaw = 360 - d_yaw

            if d_yaw > 5:
                if lower_limit < -135 and self.my_yaw > 90:
                    self.rover.publish(self.left)
                    self.status_stuff("turning left hard")            
                elif upper_limit > 135 and self.my_yaw < -90:
                    self.rover.publish(self.right)
                    self.status_stuff("turning right hard")
                elif self.my_yaw > upper_limit:
                    self.rover.publish(self.left)
                    self.status_stuff("turning left")
                elif self.my_yaw < lower_limit:
                    self.rover.publish(self.right)
                    self.status_stuff("turning right")
                else:
                    self.rover.publish(self.forward)
                    self.status_stuff("forward")

            else:
                self.rover.publish(self.forward)
                self.status_stuff("forward")
                    
            sleep(0.2)
            rclpy.spin_once(self)

        self.status_stuff("Reached Target Coordinates")
        self.autonomous_on = False
        self.rover.publish(self.stop)

def main():
    rclpy.init()
    autonomous = Autonomous()

    rclpy.spin(autonomous)

    autonomous.destroy_node()
    rclpy.shutdowm()

if __name__ == "__main__":
    main()

        
