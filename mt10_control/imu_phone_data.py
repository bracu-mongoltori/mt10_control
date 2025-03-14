import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import websocket
import json
import threading
import numpy as np

class IMUWebSocketNode(Node):
    def __init__(self):
        super().__init__('imu_websocket_node')
        self.publisher_ = self.create_publisher(Imu, '/imu_data', 5)
        self.ws_url = "ws://192.168.0.105:8080/sensor/connect?type=android.sensor.orientation"
        self.start_websocket()
    
    def start_websocket(self):
        def on_message(ws, message):
            try:
                values = json.loads(message)['values']
                imu_msg = Imu()
                imu_msg.header.stamp = self.get_clock().now().to_msg()
                imu_msg.header.frame_id = "imu_link"
                
                # Set orientation (assuming values represent roll, pitch, yaw)
                quaternion = self.euler_to_quaternion(values[0], values[1], values[2])
                imu_msg.orientation.x = quaternion[0]
                imu_msg.orientation.y = quaternion[1]
                imu_msg.orientation.z = quaternion[2]
                imu_msg.orientation.w = quaternion[3]
                
                self.publisher_.publish(imu_msg)
                # self.get_logger().info(f'Published IMU Data: roll={values[0]}, pitch={values[1]}, yaw={values[2]}')
            except KeyError:
                self.get_logger().error("Invalid message format")
            except json.JSONDecodeError:
                self.get_logger().error("Failed to decode JSON")
        
        def on_error(ws, error):
            self.get_logger().error(f'WebSocket error: {error}')
        
        def on_close(ws, close_code, reason):
            self.get_logger().warn(f'Connection closed: {reason}')
        
        def on_open(ws):
            self.get_logger().info("Connected to WebSocket")
        
        def run_ws():
            ws = websocket.WebSocketApp(
                self.ws_url,
                on_open=on_open,
                on_message=on_message,
                on_error=on_error,
                on_close=on_close
            )
            ws.run_forever()
        
        thread = threading.Thread(target=run_ws, daemon=True)
        thread.start()
    
    def euler_to_quaternion(self, roll, pitch, yaw):
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)

        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        qw = cr * cp * cy + sr * sp * sy
        return [qx, qy, qz, qw]


def main(args=None):
    rclpy.init(args=args)
    node = IMUWebSocketNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
