import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import sys

class MoveToGoal(Node):
    def __init__(self, target_x, target_y, target_theta):
        super().__init__('move_to_goal')

        self.target_x = target_x
        self.target_y = target_y
        self.target_theta = target_theta

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0

        # Создаем подписку на топик pose
        self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)

        # Создаем публикацию на топик cmd_vel
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.move_turtle)

    def pose_callback(self, msg):
        # Обновляем текущие координаты и угол
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_theta = msg.theta

    def move_turtle(self):
        msg = Twist()

        # Используем текущие координаты черепахи
        current_x = self.current_x
        current_y = self.current_y
        current_theta = self.current_theta

        # Расчет расстояния до цели
        distance = math.sqrt((self.target_x - current_x) ** 2 + (self.target_y - current_y) ** 2)

        if distance > 0.01:  # Если черепаха не достигла цели
            desired_angle = math.atan2(self.target_y - current_y, self.target_x - current_x)
            angle_diff = desired_angle - current_theta

            # Приводим угол в диапазон [-pi, pi]
            angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi

            # Устанавливаем линейную скорость
            msg.linear.x = min(1.0, distance)  # Ограничиваем скорость до 1.0

            # Устанавливаем угловую скорость
            msg.angular.z = angle_diff

            self.publisher.publish(msg)
            self.get_logger().info(f'Moving towards: ({self.target_x}, {self.target_y}) with linear speed: {msg.linear.x} and angular speed: {msg.angular.z}')
        else:
            msg.linear.x = 0.0  # Остановка черепахи
            msg.angular.z = 0.0
            self.publisher.publish(msg)
            self.get_logger().info('Reached goal!')
            rclpy.shutdown()  # Останавливаем узел после достижения цели

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 4:
        print("Usage: ros2 run move_to_goal move_to_goal <target_x> <target_y> <target_theta>")
        return

    target_x = float(sys.argv[1])
    target_y = float(sys.argv[2])
    target_theta = float(sys.argv[3])

    move_to_goal = MoveToGoal(target_x, target_y, target_theta)
    rclpy.spin(move_to_goal)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
