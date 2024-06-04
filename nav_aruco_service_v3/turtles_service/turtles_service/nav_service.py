import rclpy
from rclpy.node import Node
from turtles_service_msgs.srv import ArucoNavigateTo
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

class NavService(Node):

    def __init__(self):
        super().__init__('nav_service')
        self.srv = self.create_service(ArucoNavigateTo, 'navigate_to_pose', self.service_callback)
        self.action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.target_reached = False
        self.current_goal_position = None
        self.current_mode = None  # 추가: 현재 모드를 저장할 변수

    def service_callback(self, request: ArucoNavigateTo.Request, response: ArucoNavigateTo.Response):
        if self.target_reached:
            response.success = False
            response.message = "Already navigating to pose"
            return response

        self.get_logger().info('Service called: Navigating to pose...')
        goal_msg = self.create_goal_pose(request.x, request.y, request.z, 0.0, 0.0, 0.0, 1.0)
        
        # Save the goal position
        self.current_goal_position = [request.x, request.y, request.z]
        self.current_mode = request.mode  # 추가: 요청된 모드를 저장

        self.action_client.wait_for_server()
        send_goal_future = self.action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(lambda future: self.goal_response_callback(future, request.aruco_id))

        response.success = True
        response.message = "NavigateToPose action called successfully"
        return response

    def goal_response_callback(self, future, aruco_id):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            self.target_reached = False
            return
        self.get_logger().info('Goal accepted')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(lambda future: self.get_result_callback(future, aruco_id))

    def get_result_callback(self, future, aruco_id):
        try:
            result = future.result().result
            self.get_logger().info('Result: {0}'.format(result))
            self.target_reached = True
            if self.current_mode == "NAVIGATION_WITH_ARUCO":
                self.get_logger().info('Target reached. Starting Aruco detection.')
                self.start_aruco_detection(aruco_id)
            else:
                self.get_logger().info('Target reached. No Aruco detection required.')
                self.target_reached = False  # Reset for next navigation
        except Exception as e:
            self.get_logger().error(f'Error in getting result: {e}')
            self.target_reached = False

    def start_aruco_detection(self, aruco_id):
        client = self.create_client(ArucoNavigateTo, 'start_aruco_detection')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Aruco detection service not available, waiting again...')
        request = ArucoNavigateTo.Request()
        request.aruco_id = aruco_id

        # 로봇의 현재 목표 위치를 서비스 요청에 포함
        if self.current_goal_position:
            request.x, request.y, request.z = self.current_goal_position
        else:
            self.get_logger().warning('Current goal position is not set, using default (0, 0, 0)')
            request.x = request.y = request.z = 0.0

        future = client.call_async(request)
        future.add_done_callback(self.aruco_detection_response_callback)

    def aruco_detection_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Aruco Detection Service response: {response.message}')
            self.target_reached = False  # Reset flag for new navigation requests
        except Exception as e:
            self.get_logger().error(f'Error in Aruco detection response: {e}')
            self.target_reached = False  # Reset flag for new navigation requests

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
    nav_service = NavService()
    try:
        rclpy.spin(nav_service)
    except KeyboardInterrupt:
        pass
    nav_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()