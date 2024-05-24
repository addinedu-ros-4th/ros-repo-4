import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from rclpy.action import ActionClient
from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from turtles_service_msgs.srv import WayPointPoses

class NavService(Node):

    def __init__(self):
        super().__init__('waypoint_service')
        self.srv = self.create_service(WayPointPoses, 'waypoint_service', self.service_callback)
        self.action_client = ActionClient(self, FollowWaypoints, '/follow_waypoints')
        self.callback_group = ReentrantCallbackGroup()

    def service_callback(self, request, response):
        self.get_logger().info('Service called: Follow Waypoints...')
        
        goal = FollowWaypoints.Goal() #PoseStamped[] poses

        # goal.number_of_
        goal_list = []

        goal.poses = goal_list
  
        # Set your goal pose here
        goal_msg1 = PoseStamped()
        goal_msg1.header.frame_id = 'map'
        goal_msg1.pose.position.x = request.x1
        goal_msg1.pose.position.y = request.y1
        goal_msg1.pose.position.z = request.z1
        goal_msg1.pose.orientation.x = 0.0
        goal_msg1.pose.orientation.y = 0.0
        goal_msg1.pose.orientation.z = 0.0
        goal_msg1.pose.orientation.w = 1.0

        goal.poses.append(goal_msg1)

        # Set your goal pose here
        goal_msg2 = PoseStamped()
        goal_msg2.header.frame_id = 'map'
        goal_msg2.pose.position.x = request.x2
        goal_msg2.pose.position.y = request.y2
        goal_msg2.pose.position.z = request.z2
        goal_msg2.pose.orientation.x = 0.0
        goal_msg2.pose.orientation.y = 0.0
        goal_msg2.pose.orientation.z = 0.0
        goal_msg2.pose.orientation.w = 1.0

        goal.poses.append(goal_msg2)

        # Set your goal pose here
        goal_msg3 = PoseStamped()
        goal_msg3.header.frame_id = 'map'
        goal_msg3.pose.position.x = request.x3
        goal_msg3.pose.position.y = request.y3
        goal_msg3.pose.position.z = request.z3
        goal_msg3.pose.orientation.x = 0.0
        goal_msg3.pose.orientation.y = 0.0
        goal_msg3.pose.orientation.z = 0.0
        goal_msg3.pose.orientation.w = 1.0

        goal.poses.append(goal_msg3)

        self.action_client.wait_for_server()
        self._send_goal_future = self.action_client.send_goal_async(goal)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        
        response.message = "Waypoint Following action called successfully"
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
