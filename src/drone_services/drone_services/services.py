import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts  # Usamos este servicio para el cálculo

class SquareService(Node):
    def __init__(self):
        super().__init__('square_service')
        self.srv = self.create_service(AddTwoInts, 'calculate_square', self.calculate_square)
        self.get_logger().info('Servicio de cuadrado listo para recibir solicitudes...')

    def calculate_square(self, request, response):
        response.sum = request.a ** 2  # Calcula el cuadrado del número
        self.get_logger().info(f'Recibido: {request.a}, enviando: {response.sum}')
        return response

def main():
    rclpy.init()
    node = SquareService()
    rclpy.spin(node)  # Mantiene el nodo en ejecución
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
