import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from turtles_service_msgs.srv import NavToPose
from turtles_service_msgs.msg import ActionClientResult


class NavService(Node):

    def __init__(self):
        super().__init__('navigation_service')
        self.srv = self.create_service(NavToPose, 'navigation_service', self.service_callback)
        self.action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.callback_group = ReentrantCallbackGroup()
        self.publisher = self.create_publisher(ActionClientResult, 'pub_action_result',10)

    def service_callback(self, request, response):
        self.get_logger().info('Service called: Navigating to pose...')
        goal_msg = NavigateToPose.Goal()
        # Set your goal pose here
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = request.x
        goal_msg.pose.pose.position.y = request.y
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = request.ori_z
        goal_msg.pose.pose.orientation.w = request.ori_w

        self.action_client.wait_for_server()
        self._send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

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
        self.get_logger().info('Result: {0}'.format(result.result))
        msg = ActionClientResult()
        msg.result = "got navigation result"
        self.publisher.publish(msg)
        
    def feedback_callback(self,feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Feedback : {0}'.format(feedback.current_pose.pose.position))
        self.get_logger().info('Feedback : {0}'.format(feedback.current_pose.pose.orientation))


def main(args=None):
    rclpy.init(args=args)
    nav_service = NavService()
    rclpy.spin(nav_service)
    rclpy.shutdown()


if __name__ == '__main__':
    main()




    

    
  