import rclpy
from rclpy.node import Node 
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from sbg_driver.msg import SbgEkfEuler, SbgEkfNav
from sbg_driver.msg import SbgGpsPos
from time import sleep
from math import radians, degrees, sin, cos, asin, sqrt, atan2
import cv2 
from cv2 import aruco
import numpy as np

MARKER_SIZE = 0.15  # Marker size in meters
RECT_WIDTH = 150    # Detection zone width in pixels
RECT_HEIGHT = 90    # Detection zone height in pixels
LINEAR_SPEED = 50.0 # Forward/backward speed
ANGULAR_SPEED = 7.0 # Turning speed
STOP_DISTANCE = 0.5 # Distance threshold in meters

class ArucoTracker(Node):
    def __init__(self, max_lin_vel=45.0, max_ang_vel=7.0):
        super().__init__("aruco_tracker")
        self.rover = self.create_publisher(Twist, "/cmd_vel", 10)
        self.orientation = self.create_subscription(SbgEkfEuler, "/sbg/ekf_euler", self.orientation_callback, 10)
        self.ar_tag_state = self.create_subscription(Bool, "/ar_tag_state", self.ar_state, 10)

        self.camera = self.create_timer(0.01, self.camera_callback)
        self.ar_tag = self.create_timer(0.01, self.look_for_tag)
        self.logger_timer = self.create_timer(0.01, self.logger)

        self.vertices = []
        self.ar_state = False

        self.forward = Twist()
        self.left = Twist()
        self.right = Twist()
        self.backward = Twist()
        self.stop = Twist()

        self.forward.linear.x = max_lin_vel
        self.left.angular.z = max_ang_vel
        self.right.angular.z = -max_ang_vel
        self.backward.linear.x = -max_lin_vel

        self.cap = cv2.VideoCapture(2)
        self.marker_center = [0]
        self.distance = 0.0
        self.dir_msg = "None"
        self.stop_threshold = 1.0
        
        # Load ArUco dictionary
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters_create()

        # self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)
                
        # Flag for goal reached
        self.goal_reached = False
        self.prev_msg = None
        
        self.get_logger().info('ArUco Tracking Node initialized')

        self.frame_width = 0.0
    

    def calculate_distance(self, marker_center, image_center, marker_size_pixels):
        focal_length_pixels = 600
        distance = (MARKER_SIZE * focal_length_pixels) / marker_size_pixels
        return distance

    def ar_state(self, msg: Bool):
        self.ar_state = msg.data

    def look_for_tag(self):
        if self.ar_state == False:
            pass
        else:
            center_x = self.frame_width // 2
            lower_limit = center_x - 50
            upper_limit = center_x + 50

            if self.marker_center[0] < upper_limit and self.marker_center[0] > lower_limit:
                if self.distance < self.stop_threshold:
                    message = self.stop
                    self.rover.publish(self.stop)
                    self.get_logger().info("Reached AR Tag")
                    self.ar_state = False
                else:
                    message = self.forward
                    self.dir_msg = "forward"
            else:
                val = center_x - self.marker_center[0]
                message = self.left if val < 0 else self.right
                self.dir_msg = "left" if val < 0 else "right"

            self.rover.publish(message)
            # self.get_logger().info(dir_msg)
    
    def orientation_callback(self, msg: SbgEkfEuler):
        self.my_yaw = degrees(msg.angle.z)
        # print(self.my_yaw)

    def logger(self):
        if self.ar_state == True:
            self.get_logger().info(f'Distance: {self.distance:.2f}m')
            self.get_logger().info(f'Motion: {self.dir_msg}')    

    def camera_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('Failed to capture frame')
            return
            
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        self.frame_width = frame.shape[1]
        # corners, ids, rejected = self.detector.detectMarkers(gray)
        corners, ids, rejected = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
        if ids is not None:
            for i in range(len(ids)):
                marker_corners = corners[i][0]
                self.marker_center = tuple(np.mean(marker_corners, axis=0).astype(int))
                marker_width_pixels = np.linalg.norm(marker_corners[0] - marker_corners[1])
                
                image_center = (frame.shape[1] // 2, frame.shape[0] // 2)
                self.distance = self.calculate_distance(self.marker_center, image_center, marker_width_pixels)

                # self.get_logger().info(f'Distance: {self.distance:.2f}m')
                # self.get_logger().info(f'Motion: {self.dir_msg}')

        else:
            self.get_logger().debug('No Aruco markers detected')

    def __del__(self):
        self.cap.release()

    def create_hexagon(self, lat, lon, radius=7.5):
        # Earth's radius in meters
        self.vertices = []
        EARTH_RADIUS = 6371.0 * 1000
        
        # Convert radius to radians (distance/earth_radius)
        rad_dist = radius / EARTH_RADIUS
        
        # List to store hexagon vertices
        
        for i in range(6):
            # Angle for each vertex (60-degree increments in radians)
            angle = radians(i * 60)
            
            # Calculate latitude of the vertex
            vertex_lat = asin(
                sin(radians(lat)) * cos(rad_dist) +
                cos(radians(lat)) * sin(rad_dist) * cos(angle)
            )
            
            # Calculate longitude of the vertex
            vertex_lon = radians(lon) + atan2(
                sin(angle) * sin(rad_dist) * cos(radians(lat)),
                cos(rad_dist) - sin(radians(lat)) * sin(vertex_lat)
            )
            
            # Convert back to degrees and add to the list
            self.vertices.append((degrees(vertex_lat), degrees(vertex_lon)))


def main():
    rclpy.init()
    ar_tracker = ArucoTracker()

    rclpy.spin(ar_tracker)

    ar_tracker.destroy_node()
    rclpy.shutdowm()

if __name__ == "__main__":
    main()
