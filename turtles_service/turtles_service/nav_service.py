import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.callback_groups import ReentrantCallbackGroup

class NavService(Node):

    def __init__(self):
        super().__init__('nav_service')
        self.srv = self.create_service(Trigger, 'turtle_service', self.service_callback)
        self.action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.callback_group = ReentrantCallbackGroup()

    def service_callback(self, request, response):
        self.get_logger().info('Service called: Navigating to pose...')
        goal_msg = NavigateToPose.Goal()

        # Set your goal pose here
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = 12.38
        goal_msg.pose.pose.position.y = -7.37
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        self.action_client.wait_for_server()
        self._send_goal_future = self.action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        
        response.success = True
        response.message = "NavigateToPose action called successfully"
        return response

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result))

def main(args=None):
    rclpy.init(args=args)
    nav_service = NavService()
    rclpy.spin(nav_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
