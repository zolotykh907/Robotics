from full_name_package.srv import FullNameSumService
from full_name_package.msg import FullNameMessage

import rclpy
from rclpy.node import Node
from full_name_package.srv import FullNameSumService

class ServiceName(Node):

    def __init__(self):
        super().__init__('service_name')
        self.srv = self.create_service(FullNameSumService, 'SummFullName', self.handle_service)

    def handle_service(self, request, response):
        response.full_name = f"{request.last_name} {request.name} {request.first_name}"
        self.get_logger().info(f"Returning full name: {response.full_name}")
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ServiceName()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

