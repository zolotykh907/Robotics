import rclpy
from rclpy.node import Node
from full_name_package.srv import FullNameSumService

class ClientName(Node):

    def __init__(self):
        super().__init__('client_name')
        self.cli = self.create_client(FullNameSumService, 'SummFullName')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')

        self.send_request()

    def send_request(self):
        # Ввод данных с консоли
        last_name = input("Enter last name: ")
        name = input("Enter name: ")
        first_name = input("Enter first name: ")

        request = FullNameSumService.Request()
        request.last_name = last_name
        request.name = name
        request.first_name = first_name

        future = self.cli.call_async(request)
        future.add_done_callback(self.handle_response)

    def handle_response(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Received full name: {response.full_name}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = ClientName()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
