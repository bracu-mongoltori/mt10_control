import cv2
import numpy as np
from cv2 import aruco
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sbg_driver.msg import SbgEkfEuler
from sbg_driver.msg import SbgGpsPos
from sbg_driver.msg import SbgEkfNav

from math import radians, degrees, sin, cos


# Marker size in meters (side length)
MARKER_SIZE = 0.15

# Rectangle dimensions
RECT_WIDTH = 150
RECT_HEIGHT = 90

# Movement parameters
LINEAR_SPEED = 50.0
ANGULAR_SPEED = 7.0
STOP_DISTANCE = 1.0  # Distance threshold in meters

# Hexagon parameters
HEX_RADIUS = 2.0  # Radius of the hexagon in meters
EARTH_RADIUS = 6371000  # Earth radius in meters


class ArucoTrackingNode(Node):
    def __init__(self):
        super().__init__('aruco_tracking_node')
        
        # Create publisher for cmd_vel
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Initialize video capture
        self.cap = cv2.VideoCapture(1)
        
        # Load ArUco dictionary
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters()
        
        # Create timer for camera callback
        self.timer = self.create_timer(0.01, self.camera_callback)  # 10Hz
        self.gps = self.create_subscription(SbgGpsPos, "/sbg/gps_pos", self.gps_callback, 10)
        
        self.my_lat = 0.0
        self.my_lon = 0.0
        self.my_yaw = 0.0
        
        # Flag for goal reached
        self.goal_reached = False
        self.prev_msg = None
        self.mode = None
        
        self.get_logger().info('ArUco Tracking Node initialized')

    def calculate_distance(self, marker_center, image_center, marker_size_pixels):
        focal_length_pixels = 600
        distance = (MARKER_SIZE * focal_length_pixels) / marker_size_pixels
        return distance

    def draw_guides(self, frame, draw_frame):
        height, width = frame.shape[:2]
        center_x, center_y = width // 2, height // 2
        
        cv2.line(draw_frame, (0, center_y), (width, center_y), (255, 255, 255), 1)
        cv2.line(draw_frame, (center_x, 0), (center_x, height), (255, 255, 255), 1)
        
        rect_x1 = center_x - RECT_WIDTH // 2
        rect_y1 = center_y - RECT_HEIGHT // 2
        rect_x2 = center_x + RECT_WIDTH // 2
        rect_y2 = center_y + RECT_HEIGHT // 2
        cv2.rectangle(draw_frame, (rect_x1, rect_y1), (rect_x2, rect_y2), (0, 255, 0), 1)
        
        return rect_x1, rect_y1, rect_x2, rect_y2
    
    def gps_callback(self, msg: SbgGpsPos):
    #def gps_callback(self, msg: SbgEkfNav):
        self.my_lat = msg.latitude
        self.my_lon = msg.longitude
        
    def orientation_callback(self, msg: SbgEkfEuler):
        self.my_yaw = degrees(msg.angle.z)
        # print(self.my_yaw)

    def get_movement_instruction(self, marker_center, rect_bounds, distance):
        # If we're within stop distance, always return "Stop"
        if distance < STOP_DISTANCE:
            return "Stop"
            
        rect_x1, _, rect_x2, _ = rect_bounds
        x, _ = marker_center
        
        if x < rect_x1:
            return "Move Left"
        elif x > rect_x2:
            return "Move Right"
        else:
            return "Move Forward"

    def publish_movement_command(self, instruction):
        msg = Twist()
        
        if instruction == "Stop":
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.prev_msg = msg
            if not self.goal_reached:
                self.goal_reached = True
                self.get_logger().info('Goal reached! Stopping robot.')
        else:
            if instruction == "Move Forward":
                msg.linear.x = LINEAR_SPEED
                msg.angular.z = 0.0
                self.prev_msg = msg
                
            elif instruction == "Move Left":
                msg.linear.x = 0.0
                msg.angular.z = -ANGULAR_SPEED
                self.prev_msg = msg
                
            elif instruction == "Move Right":
                msg.linear.x = 0.0
                msg.angular.z = ANGULAR_SPEED
                self.prev_msg = msg
        if self.prev_msg != msg:
            self.vel_publisher.publish(msg)

    def calculate_hexagon_vertices(self, lat, lon):
        vertices = []
        for i in range(6):
            angle = radians(60 * i)
            delta_lat = (HEX_RADIUS / EARTH_RADIUS) * cos(angle)
            delta_lon = (HEX_RADIUS / (EARTH_RADIUS * cos(radians(lat)))) * sin(angle)
            vertex_lat = lat + degrees(delta_lat)
            vertex_lon = lon + degrees(delta_lon)
            vertices.append((vertex_lat, vertex_lon))
        return vertices

    def camera_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('Failed to capture frame')
            return
            
        display_frame = frame.copy()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        corners, ids, rejected = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
        rect_bounds = self.draw_guides(frame, display_frame)
        
        if ids is not None:
            for i in range(len(ids)):
                marker_corners = corners[i][0]
                marker_center = tuple(np.mean(marker_corners, axis=0).astype(int))
                marker_width_pixels = np.linalg.norm(marker_corners[0] - marker_corners[1])
                
                image_center = (frame.shape[1] // 2, frame.shape[0] // 2)
                distance = self.calculate_distance(marker_center, image_center, marker_width_pixels)
                
                instruction = self.get_movement_instruction(marker_center, rect_bounds, distance)
                
                # Log the instruction and distance
                self.get_logger().info(f'Distance: {distance:.2f}m, Instruction: {instruction}')
                
                # Publish movement command
                self.publish_movement_command(instruction)
                
                # Draw marker and information on display frame
                aruco.drawDetectedMarkers(display_frame, corners)
                cv2.rectangle(display_frame, 
                            (marker_center[0] - 5, marker_center[1] - 5),
                            (marker_center[0] + 5, marker_center[1] + 5), 
                            (0, 0, 255), -1)
                
                # Update display text color based on distance
                text_color = (0, 0, 255) if distance < STOP_DISTANCE else (0, 255, 0)
                
                text = f"ID: {ids[i][0]} Distance: {distance:.2f}m"
                cv2.putText(display_frame, text, 
                           (10, 30 + i * 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 
                           0.6, text_color, 2)
                cv2.putText(display_frame, f"Movement: {instruction}", 
                           (10, frame.shape[0] - 20), 
                           cv2.FONT_HERSHEY_SIMPLEX, 
                           0.8, text_color, 2)
        else:
            # ArUco not detected, calculate hexagon vertices and do a 360 search
            
            
            if self.mode == "rotating":
                cur_yaw = self.my_yaw
                
                if cur_yaw == (prev_yaw-5):
                    self.get_logger().info("360 search complete, did not find tag")
                    self.mode = "point_follow"
            else:
                msg = Twist()
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                
                self.vel_publisher.publish(msg)
                
                self.get_logger().info("AR tag not found. Initiating search")
                self.mode = "360-search-init"
                
                if self.mode == "360-search-init":
                    prev_yaw = self.my_yaw
                    msg.angular.z = ANGULAR_SPEED
                    self.prev_msg = msg
                    self.get_logger().info("Rotating --> ")
                    self.mode = "rotating"
                    
                    
                if self.mode== "point_follow":
                    current_lat = 37.7749  # Replace with GPS latitude
                    current_lon = -122.4194  # Replace with GPS longitude
                    vertices = self.calculate_hexagon_vertices(current_lat, current_lon)
                    self.get_logger().info("Hexagon vertices (lat, lon):")
                    for i, vertex in enumerate(vertices):
                        self.get_logger().info(f"Vertex {i + 1}: {vertex}")
                
            if msg!= self.prev_msg:
                self.vel_publisher.publish(msg)
                    
                    
                    
            
            
            
            
        
        cv2.imshow('ArUco Marker Detection', display_frame)
        cv2.waitKey(1)

    def __del__(self):
        self.cap.release()
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = ArucoTrackingNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
