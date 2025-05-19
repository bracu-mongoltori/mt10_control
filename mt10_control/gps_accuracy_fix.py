#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sbg_driver.msg import SbgGpsPos
from ublox_msgs.msg import NavPVT


class DualGpsListener(Node):

    def __init__(self):
        super().__init__('best_gps_listener')

        # Subscribers
        self.create_subscription(SbgGpsPos, '/sbg/gps_pos',    self.sbg_callback, 10)
        self.create_subscription(NavPVT,       '/navpvt',      self.sparkfun_callback, 10)
        self.create_subscription(SbgGpsPos,    '/m9n_gps',     self.m9n_callback, 10)

        # Publisher
        self.publisher = self.create_publisher(SbgGpsPos, '/best_gps_acc', 10)

        # Storage for latest messages + timestamps
        self.latest_sbg = None
        self.latest_sparkfun = None
        self.latest_m9n = None
        self.last_sbg_time = None
        self.last_sparkfun_time = None
        self.last_m9n_time = None

        # Freshness threshold (seconds)
        self.max_age = 1.0

        # Timer: runs selection logic at 10 Hz regardless of which topic arrives
        self.timer = self.create_timer(0.1, self.select_and_publish)

        self.get_logger().info('BEST GPS listener with timer started.')

    def sbg_callback(self, msg: SbgGpsPos):
        self.latest_sbg = msg
        self.last_sbg_time = self.get_clock().now()

    def sparkfun_callback(self, msg: NavPVT):
        self.latest_sparkfun = msg
        self.last_sparkfun_time = self.get_clock().now()

    def m9n_callback(self, msg: SbgGpsPos):
        self.latest_m9n = msg
        self.last_m9n_time = self.get_clock().now()

    def select_and_publish(self):
        now = self.get_clock().now()
        candidates = []

        # SBG
        if self.latest_sbg and (now - self.last_sbg_time).nanoseconds * 1e-9 < self.max_age:
            acc = self.latest_sbg.position_accuracy.x
            candidates.append(('sbg', self.latest_sbg, acc))

        # SparkFun
        if self.latest_sparkfun and (now - self.last_sparkfun_time).nanoseconds * 1e-9 < self.max_age:
            acc = self.latest_sparkfun.h_acc / 1000.0
            # Wrap NavPVT into SbgGpsPos msg
            msg = SbgGpsPos()
            msg.latitude  = self.latest_sparkfun.lat / 1e7
            msg.longitude = self.latest_sparkfun.lon / 1e7
            msg.position_accuracy.x = acc
            msg.position_accuracy.y = acc
            candidates.append(('sparkfun', msg, acc))

        # M9N
        if self.latest_m9n and (now - self.last_m9n_time).nanoseconds * 1e-9 < self.max_age:
            acc = self.latest_m9n.position_accuracy.x
            candidates.append(('m9n', self.latest_m9n, acc))

        # Filter by max acceptable accuracy
        candidates = [(n, m, a) for n, m, a in candidates if a <= 0.5]

        if not candidates:
            self.get_logger().warn('No valid GPS data to publish.')
            zero_data = SbgGpsPos()
            self.publisher.publish(zero_data)
            return

        # Select best
        best = min(candidates, key=lambda tup: tup[2])
        self.get_logger().info(f'Publishing from {best[0]} with accuracy {best[2]:.3f} m')
        self.publisher.publish(best[1])


def main(args=None):
    rclpy.init(args=args)
    node = DualGpsListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()









# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node

# from sbg_driver.msg import SbgGpsPos
# from ublox_msgs.msg import NavPVT


# class DualGpsListener(Node):

#     def __init__(self):
#         super().__init__('dual_gps_listener')

#         # Subscriber to /sbg/gps_pos
#         self.subscription_gps_pos = self.create_subscription(
#             SbgGpsPos,
#             '/sbg/gps_pos',
#             self.gps_pos_callback,
#             10
#         )
#         self.subscription_gps_pos  # prevent unused variable warning

#         # Subscriber to /navpvt
#         self.subscription_navpvt = self.create_subscription(
#             NavPVT,
#             '/navpvt',
#             self.navpvt_callback,
#             10
#         )
#         self.subscription_navpvt  # prevent unused variable warning

#         self.get_logger().info('Dual GPS listener node started.')
#         self.sparkfun_msg = NavPVT()
#         self.publisher = self.create_publisher(SbgGpsPos, '/best_gps_acc', 10)

#     def gps_pos_callback(self, msg: SbgGpsPos):
#         accuracy_sbg_x = msg.position_accuracy.x
#         accuracy_sbg_y = msg.position_accuracy.y
#         accuracy_sparkfun_x = self.sparkfun_msg.h_acc / 1000.0
#         accuracy_sparkfun_x = self.sparkfun_msg.h_acc / 1000.0

#         if accuracy_sbg_x <= accuracy_sparkfun_x:
#             self.publisher.publish(msg)
#         else:
#             msg.latitude = self.sparkfun_msg.lat / 1e-7
            








#     def navpvt_callback(self, msg: NavPVT):
#         self.sparkfun_msg = msg


# def main(args=None):
#     rclpy.init(args=args)
#     node = DualGpsListener()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()