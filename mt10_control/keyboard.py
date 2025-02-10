import evdev
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__("twist_publisher")
        self.publisher_ = self.create_publisher(Twist, "cmd_vel", 10)

    def publish_twist(self, device):
        # Polling the input device for events
        current_key_pressed = None
        linear = 0.0
        angular = 0.0
        twist_msg = Twist()
        self.get_logger().info(f"Linear: {linear}, Angular: {angular}")

        for event in device.read_loop():
            if event.code == evdev.ecodes.KEY_W and event.value == 1:
                current_key_pressed = evdev.ecodes.KEY_W
                twist_msg.linear.x = linear
                twist_msg.angular.z = 0.0
                self.publisher_.publish(twist_msg)

            if event.code == evdev.ecodes.KEY_A and event.value == 1:
                current_key_pressed = evdev.ecodes.KEY_A
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = angular
                self.publisher_.publish(twist_msg)

            if event.code == evdev.ecodes.KEY_S and event.value == 1:
                current_key_pressed = evdev.ecodes.KEY_S
                twist_msg.linear.x = -linear
                twist_msg.angular.z = 0.0
                self.publisher_.publish(twist_msg)

            if event.code == evdev.ecodes.KEY_D and event.value == 1:
                current_key_pressed = evdev.ecodes.KEY_D
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = -angular
                self.publisher_.publish(twist_msg)

            if event.code == evdev.ecodes.KEY_Q and event.value == 1:
                linear += 5.0
                self.get_logger().info(f"Linear: {linear}, Angular: {angular}")

            if event.code == evdev.ecodes.KEY_Z and event.value == 1 and linear > 0.0:
                linear -= 5.0
                self.get_logger().info(f"Linear: {linear}, Angular: {angular}")

            if event.code == evdev.ecodes.KEY_E and event.value == 1:
                angular += 5.0
                self.get_logger().info(f"Linear: {linear}, Angular: {angular}")

            if event.code == evdev.ecodes.KEY_C and event.value == 1:
                angular -= 5.0
                self.get_logger().info(f"Linear: {linear}, Angular: {angular}")

            if event.code == evdev.ecodes.KEY_X and event.value == 1:
                # Stop rover regardless of what ever is pressed
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = 0.0
                self.publisher_.publish(twist_msg)

            if event.code == current_key_pressed and event.value == 0:
                # When key is released the rover will stop moving
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = 0.0
                self.publisher_.publish(twist_msg)


def main(args=None):
    # THIS CODE DOES NOT ACCOUNT FOR THE CASE WHERE  W IS HELD AND THEN A IS ORESSED AND RELEASED IT SHOULD GO BACK TO W
    device_name = "ITE Tech. Inc. ITE Device(8176) Keyboard"
    device = None

    # Iterate through all input devices and find the one matching the name
    for device_path in evdev.list_devices():
        dev_name = evdev.InputDevice(device_path).name
        if device_name in dev_name:
            device = evdev.InputDevice(device_path)
            break

    if device is None:
        print(f"Device with name '{device_name}' not found.")
        exit()

    print(f"Listening to {device_name}")

    rclpy.init(args=args)
    twist_publisher_node = KeyboardTeleop()
    twist_publisher_node.publish_twist(device)
    # rclpy.spin(twist_publisher_node) # not needed when publishing that is why it is commented but keeping it here just in case
    twist_publisher_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
