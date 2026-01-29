import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts  # Servicio predefinido

class SquareClient(Node):
    def __init__(self):
        super().__init__('square_client')
        self.client = self.create_client(AddTwoInts, 'calculate_square')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando servicio...')

    def send_request(self, number):
        request = AddTwoInts.Request()
        request.a = number
        future = self.client.call_async(request)
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Respuesta del servicio: {response.sum}')
        except Exception as e:
            self.get_logger().error(f'Error al llamar al servicio: {str(e)}')

def main():
    rclpy.init()
    node = SquareClient()

    # Pedir el número al usuario
    try:
        number = int(input("Ingrese un número para calcular su cuadrado: "))
        node.send_request(number)
        rclpy.spin(node)
    except ValueError:
        print("Error: Ingrese un número válido.")

    node.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
    main()
