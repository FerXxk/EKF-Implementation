import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

class DroneSubscriber(Node):
    def __init__(self):
        super().__init__('drone_subscriber')
        self.subscription = self.create_subscription(
            NavSatFix,
            'drone_gps',
            self.position_callback,
            10)
        self.subscription  # Evita que el garbage collector elimine la suscripción

    def position_callback(self, msg):
        self.get_logger().info(f'Recibido GPS: lat={msg.latitude}, lon={msg.longitude}, alt={msg.altitude}')

def main(args=None):
    rclpy.init(args=args)
    node = DroneSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
