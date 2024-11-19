import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from sbg_driver.msg import SbgEkfEuler
from sbg_driver.msg import SbgGpsPos
from time import sleep
from math import radians, sin, cos, asin, sqrt, atan2, pi
from haversine import haversine as hs


class Auto(Node):
    def __init__(self, max_lin_vel=50.0, max_ang_vel=6.0, yaw_threshold=3.0, distance_threshold=0.5):
        super().__init__("autonomous")
        self.autonomous = self.create_subscription(SbgGpsPos, "/coordinates", self.target_coordinates, 10) # Coordinates reciveing msg
        self.send_velocity = self.create_publisher(Twist, "/cmd_vel", 10)  # Sending velocity to /cmd_vel
        self.gps = self.create_subscription(SbgGpsPos, "/sbg/gps_pos", self.gps_callback, 10) # Recieving GPS data
        self.orientation = self.create_subscription(SbgEkfEuler, "/sbg/ekf_euler", self.orientation_callback, 10) # Recieving orientation data
        self.status_pub = self.create_publisher(String, "/status", 10) # For log and staus publihing
        self.autonomous_status = self.create_publisher(Bool, "/autonomous_status", 10) # For publishing status of autonomous navigation
        self.status_timer = self.create_timer(0.5, self.status_stuff) # Setting frequency of status publishing
        self.autonomous_timer = self.create_timer(0.2, self.autonomous_callback) # Setting frequency of autonomous navigation

        self.autonomous_on = False # Autonomous navigation status is off on initialization
        self.bearing_angle_reached = False # Bearing angle reached status is False on initialization

        # Creating Cordiantes and orientation of rover
        self.current_latitude = 0,0
        self.current_longitude = 0.0
        self.current_yaw = 0.0

        # Creating target coordinates
        self.target_latitude = 0.0
        self.target_longitude = 0.0

        # Setting yaw and distance thresholds
        self.yaw_threshold = yaw_threshold
        self.distance_threshold = distance_threshold

        # Creating Twist messages for rover movement
        self.forward = Twist()
        self.left = Twist()
        self.right = Twist()
        self.backward = Twist()
        self.stop = Twist()

        # Setting velocities for rover movement

        # Forward--> all positive linear velocity + 0 angular velocity
        self.forward.linear.x = max_lin_vel
        self.forward.angular.z = 0.0

        # Left --> positive angular velocity + 0 linear velocity
        self.left.angular.z = max_ang_vel
        self.left.linear.x = 0.0

        # Right --> negative angular velocity + 0 linear velocity
        self.right.angular.z = -max_ang_vel
        self.right.linear.x = 0.0

        # Stop --> 0 linear velocity + 0 angular velocity
        self.stop.linear.x = 0.0
        self.stop.angular.z = 0.0

        # For checking if the previous message is the same as the current message
        self.prev_msg = None

        # Starting the node
        self.get_logger().info("Node initialized. Begin.")

    def distance_from_target(self, c_lat, c_lon, t_lat, t_lon):
        R = 6371.0 # Radius of the Earth
        c_lat = c_lat #current latitude
        c_lon = c_lon #current longitude
        t_lat = t_lat #target latitude
        t_lon = t_lon #target longitude

        # dlon = t_lon - c_lon
        # dlat = t_lat - c_lat

        # # Haversine formula

        # a = sin(dlat/2)**2 + cos(c_lat) * cos(t_lat) * sin(dlon/2)**2
        # c = 2 * asin(sqrt(a))

        # return R * c *1000 # Distance in meters

        return hs((c_lat, c_lon),(t_lat, t_lon)) * 1000 # Distance in meters

    def calculate_bearing_angle(self, c_lat, c_lon, t_lat, t_lon):
        current_lat, current_lon, target_lat, target_lon = map(radians, [c_lat, c_lon, t_lat, t_lon]) # Convert to radians

        longitudinal_difference = target_lon - current_lon

        y= sin(longitudinal_difference) * cos(target_lat)
        x= cos(current_lat) * sin(target_lat) - sin(current_lat) * cos(target_lat) * cos(longitudinal_difference)

        bearing_angle = atan2(y,x)

        bearing_angle = (bearing_angle * (180/pi))%360

        bearing_angle = round(bearing_angle, 2)

        return bearing_angle

    def gps_callback(self, msg: SbgGpsPos): # Callback for GPS data receiving
        self.current_latitude = round(msg.latitude, 5)
        self.current_longitude = round(msg.longitude, 5)

    def orientation_callback(self, msg: SbgEkfEuler): # Callback for orientation data receiving
        self.current_yaw = (msg.angle.z * (180/pi))%360
        self.current_yaw = round(self.current_yaw, 2)

    def status_stuff(self, msg=None): # For publishing status of the rover
        status_pub = String()
        status_bool_pub = Bool()
        if self.autonomous_on == False:
            bool_msg = False
            msg = "Rover autonomous navigation not active"
        else:
            bool_msg = True
        status_pub.data = str(msg)
        status_bool_pub.data = bool_msg
        self.status_pub.publish(status_pub)
        self.get_logger().info(msg)
        self.autonomous_status.publish(status_bool_pub)

    def target_coordinates(self, msg: SbgGpsPos): # Callback for target coordinates data receiving
        self.autonomous_on = True
        self.target_latitude = msg.latitude
        self.target_longitude = msg.longitude
        self.get_logger().info("Beginning Navigation")
        self.get_logger().info(f"Target coordinates set to: {self.target_latitude}, {self.target_longitude}")

    def autonomous_callback(self): # Main autonomous navigation function
        msg = None
        if self.autonomous_on == False:
            pass
        else:
            self.get_logger().info(f"Current latitude, longitude: {self.current_latitude}, {self.current_longitude}")
            self.get_logger().info(f"Current yaw: {self.current_yaw}")
            remaining_distance = self.distance_from_target(self.current_latitude, self.current_longitude, self.target_latitude, self.target_longitude)
            self.get_logger().info(f"Distance to target: {remaining_distance} meters")
            self.bearing_angle = self.calculate_bearing_angle(self.current_latitude, self.current_longitude, self.target_latitude, self.target_longitude)
            self.get_logger().info(f"Bearing angle: {self.bearing_angle} degrees")
            yaw_difference = abs(self.bearing_angle - self.current_yaw)
            self.get_logger().info(f"Yaw difference: {yaw_difference} degrees")
            log= "this is the log"
            if remaining_distance < self.distance_threshold:
                print('Target reached')
                msg = self.stop
                log = "reached"
                self.autonomous_on = False
            else:
                print(f"check {self.bearing_angle}")
                if yaw_difference <= self.yaw_threshold+5.0: # If the yaw difference is less than the threshold, move forward
                    msg = self.stop
                    log = f"Stop, test done. Distance to target: {remaining_distance} meters"
                else:
                    if self.bearing_angle > self.current_yaw:
                        msg = self.right
                        log = f"Turning right. {yaw_difference} degrees to go"
                    elif self.bearing_angle < self.current_yaw:
                        msg = self.left
                        log = f"Turning left. {yaw_difference} degrees to go"
            if msg == self.prev_msg:
                self.get_logger().info(log)
                pass
            else:
                self.send_velocity.publish(msg) # Publish the velocity to /cmd_vel
                self.prev_msg = msg
                self.get_logger().info(log)

def main(args=None):
    rclpy.init(args=args)
    autonomous = Auto()

    rclpy.spin(autonomous)

    autonomous.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
