import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus

class DronePublisher(Node):
    def __init__(self):
        super().__init__('drone_publisher')
        self.publisher_ = self.create_publisher(NavSatFix, 'drone_gps', 10)
        self.timer = self.create_timer(1.0, self.publish_position)  # Publica cada 1 segundo
        self.latitude = 37.7749   # Ejemplo: San Francisco, CA
        self.longitude = -122.4194
        self.altitude = 10.0  # 10 metros de altura

    def publish_position(self):
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "gps_frame"
        msg.status.status = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS
        msg.latitude = self.latitude
        msg.longitude = self.longitude
        msg.altitude = self.altitude

        # Covarianza simulada (valores típicos para GPS)
        msg.position_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

        self.get_logger().info(f'Publicando GPS: lat={msg.latitude}, lon={msg.longitude}, alt={msg.altitude}')
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DronePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

