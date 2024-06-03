import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
import math
import time

class NewPositionSubscriber(Node):

    def __init__(self):
        super().__init__('new_position_subscriber')
        self.subscription = self.create_subscription(
            Point,
            'new_position',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(Twist, '/base_controller/cmd_vel_unstamped', 10)
        self.subscription  # prevent unused variable warning
        self.target_position = None

    def listener_callback(self, msg):
        self.get_logger().info(f'New position received: x={msg.x}, y={msg.y}, z={msg.z}')
        self.target_position = msg
        self.navigate_to_pose()

    def navigate_to_pose(self):
        if self.target_position is None:
            return

        twist_msg = Twist()
        rate = self.create_rate(10)  # 10 Hz

        while rclpy.ok():
            # Assuming current position as origin (0,0,0), calculate the direction and distance to target
            distance = math.sqrt(self.target_position.x ** 2 + self.target_position.y ** 2)
            if distance < 0.05:  # If within 5 cm of the target position, stop
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = 0.0
                self.publisher.publish(twist_msg)
                self.get_logger().info('Reached the target position.')
                break

            # Calculate proportional linear velocity
            twist_msg.linear.x = min(0.2, distance)  # Limit max speed to 0.2 m/s
            twist_msg.linear.y = 0.0
            twist_msg.linear.z = 0.0

            # Calculate proportional angular velocity
            angle_to_target = math.atan2(self.target_position.y, self.target_position.x)
            twist_msg.angular.z = angle_to_target * 2  # Proportional control for rotation

            self.publisher.publish(twist_msg)
            self.get_logger().info(f'Publishing cmd_vel to move towards new position: linear.x={twist_msg.linear.x}, angular.z={twist_msg.angular.z}')
            rate.sleep()

        # Once arrived, ensure the robot stops
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.publisher.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    new_position_subscriber = NewPositionSubscriber()
    try:
        rclpy.spin(new_position_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        new_position_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
