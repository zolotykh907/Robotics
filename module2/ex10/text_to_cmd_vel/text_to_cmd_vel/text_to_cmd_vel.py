import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class TextToCmdVelNode(Node):
    def __init__(self):
        super().__init__('text_to_cmd_vel')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscription = self.create_subscription(
            String,
            'cmd_text',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, msg):
        twist = Twist()
        if msg.data == 'turn_right':
            twist.angular.z = -1.5
        elif msg.data == 'turn_left':
            twist.angular.z = 1.5
        elif msg.data == 'move_forward':
            twist.linear.x = 1.0
        elif msg.data == 'move_backward':
            twist.linear.x = -1.0
        else:
            self.get_logger().info(f'Unknown command: {msg.data}')
            return
        self.publisher_.publish(twist)
        self.get_logger().info(f'Publishing: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = TextToCmdVelNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
