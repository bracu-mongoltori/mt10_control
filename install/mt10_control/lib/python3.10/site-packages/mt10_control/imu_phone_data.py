import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import websocket
import json
import threading

class IMUWebSocketNode(Node):
    def __init__(self):
        super().__init__('imu_websocket_node')
        self.publisher_ = self.create_publisher(Float32MultiArray, '/imu_data', 10)
        self.ws_url = "ws://192.168.0.105:8080/sensor/connect?type=android.sensor.orientation"
        self.start_websocket()
    
    def start_websocket(self):
        def on_message(ws, message):
            try:
                values = json.loads(message)['values']
                msg = Float32MultiArray()
                msg.data = [values[0], values[1], values[2]]
                self.publisher_.publish(msg)
                self.get_logger().info(f'Published IMU Data: x={values[0]}, y={values[1]}, z={values[2]}')
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


def main(args=None):
    rclpy.init(args=args)
    node = IMUWebSocketNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
