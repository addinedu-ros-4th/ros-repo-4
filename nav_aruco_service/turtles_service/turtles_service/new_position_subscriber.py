import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Point, PoseStamped

class NewPositionSubscriber(Node):

    def __init__(self):
        super().__init__('new_position_subscriber')
        self.subscription = self.create_subscription(
            Point,
            'new_position',
            self.listener_callback,
            10)
        self.action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.target_reached = False
        self.current_goal_position = None

    def listener_callback(self, msg):
        self.get_logger().info(f'New position received: x={msg.x}, y={msg.y}, z={msg.z}')
        self.current_goal_position = [msg.x, msg.y, msg.z]

        # 목표 위치 수신 시 target_reached 플래그 초기화
        self.target_reached = False

        # 새로운 목표 위치로 이동
        self.navigate_to_pose(msg.x, msg.y, msg.z)

    def navigate_to_pose(self, x, y, z):
        if self.target_reached:
            self.get_logger().info('Already navigating to pose')
            return

        self.get_logger().info('Navigating to new position...')
        goal_msg = self.create_goal_pose(x, y, z, 0.0, 0.0, 0.0, 1.0)

        self.action_client.wait_for_server()
        send_goal_future = self.action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            self.target_reached = False
            return
        self.get_logger().info('Goal accepted')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        try:
            result = future.result().result
            self.get_logger().info(f'Result: {result}')
            self.target_reached = True
            self.get_logger().info('Target reached.')
        except Exception as e:
            self.get_logger().error(f'Error in getting result: {e}')
            self.target_reached = False

    def create_goal_pose(self, x, y, z, ox, oy, oz, ow):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = z
        goal_msg.pose.pose.orientation.x = ox
        goal_msg.pose.pose.orientation.y = oy
        goal_msg.pose.pose.orientation.z = oz
        goal_msg.pose.pose.orientation.w = ow
        return goal_msg

def main(args=None):
    rclpy.init(args=args)
    new_position_subscriber = NewPositionSubscriber()
    try:
        rclpy.spin(new_position_subscriber)
    except KeyboardInterrupt:
        pass
    new_position_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

