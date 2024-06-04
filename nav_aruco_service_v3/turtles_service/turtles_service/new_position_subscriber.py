import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
import math

class NewPositionSubscriber(Node):

    def __init__(self):
        super().__init__('new_position_subscriber')
        self.subscription = self.create_subscription(
            Point,
            'new_position',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(Twist, '/base_controller/cmd_vel_unstamped', 10)
        self.target_position = None
        self.current_position = Point()  # 현재 위치를 저장할 변수 추가
        self.moving = False

        # 타이머 콜백을 사용하여 주기적으로 이동 명령을 업데이트 (주기 1초로 변경)
        self.timer = self.create_timer(1.0, self.navigate_to_pose)

    def listener_callback(self, msg):
        self.get_logger().info(f'New position received: x={msg.x}, y={msg.y}, z={msg.z}')
        self.target_position = msg
        self.moving = True

    def navigate_to_pose(self):
        if self.target_position is None or not self.moving:
            return

        twist_msg = Twist()

        # 목표 위치까지의 거리 계산 (x 축만 사용)
        distance = abs(self.target_position.x - self.current_position.x)
        if distance < 0.05:  # 목표 위치에 5cm 이내로 도달하면 멈춤
            self.stop_robot()
            self.get_logger().info('Reached the target position.')
            self.target_position = None  # 목표 위치 초기화
            self.moving = False  # 이동 완료 후 플래그 초기화
            return

        # 선형 속도 계산 (비례 제어)
        twist_msg.linear.x = min(0.5, distance)  # 최대 속도 0.5 m/s로 제한

        self.publisher.publish(twist_msg)
        self.get_logger().info(f'Publishing cmd_vel to move towards new position: linear.x={twist_msg.linear.x}')

        # 현재 위치 업데이트 (간단히 하기 위해 이동 속도에 비례하여 업데이트)
        self.current_position.x += twist_msg.linear.x * 1.0  # 주기를 1초로 설정했으므로 1.0을 곱함

    def stop_robot(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.publisher.publish(twist_msg)
        self.get_logger().info('Stopping the robot.')

def main(args=None):
    rclpy.init(args=args)
    new_position_subscriber = NewPositionSubscriber()
    try:
        rclpy.spin(new_position_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        new_position_subscriber.stop_robot()
        new_position_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
