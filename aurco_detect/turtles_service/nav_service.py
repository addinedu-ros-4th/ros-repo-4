import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from rclpy.callback_groups import ReentrantCallbackGroup
from tf_transformations import quaternion_from_euler
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import threading
import time

class NavService(Node):

    def __init__(self):
        super().__init__('nav_service')
        self.srv = self.create_service(Trigger, 'turtle_service', self.service_callback)
        self.action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.callback_group = ReentrantCallbackGroup()

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.image_sub = self.create_subscription(
            Image, 'camera_image', self.image_callback, qos_profile)
        self.cmd_vel_pub = self.create_publisher(Twist, '/base_controller/cmd_vel_unstamped', 10)

        self.br = CvBridge()
        self.target_reached = False
        self.aruco_found = False
        self.rotating = False

        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        self.intrinsic_camera = np.array([[933.15867, 0, 657.59],
                                          [0, 933.1586, 400.36993],
                                          [0, 0, 1]])
        self.distortion = np.array([-0.43948, 0.18514, 0, 0])

        self.create_timer(2.0, self.rotation_loop)  # 타이머를 이용해 회전 간격을 설정

        self.image_queue = []
        self.processing_thread = threading.Thread(target=self.process_images)
        self.processing_thread_running = True
        self.processing_thread.start()

    def service_callback(self, request, response):
        if self.target_reached:
            response.success = False
            response.message = "Already navigating to pose"
            return response

        self.get_logger().info('Service called: Navigating to pose...')
        goal_msg = self.create_goal_pose(0.8933855295181274, 0.0037445751950144768,  0.002471923828125, 0.0, 0.0, 0.0, 1.0)

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
        if result:
            self.target_reached = True
            self.get_logger().info('Target reached. Starting ArUco marker detection.')
            self.rotating = True

    def rotation_loop(self):
        if self.rotating and not self.aruco_found:
            twist = Twist()
            twist.angular.z = 0.5
            self.cmd_vel_pub.publish(twist)
            self.get_logger().info('Rotating...')

    def stop_rotation(self):
        self.get_logger().info('Stopping rotation...')
        self.rotating = False
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

    def image_callback(self, msg):
        self.image_queue.append(msg)

    def process_images(self):
        while self.processing_thread_running:
            if self.image_queue:
                msg = self.image_queue.pop(0)
                start_time = time.time()
                if not self.target_reached:
                    continue

                frame = self.br.imgmsg_to_cv2(msg, 'bgr8')
                self.get_logger().info('Processing camera image...')
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

                if ids is not None and len(ids) > 0:
                    self.get_logger().info(f'ArUco marker detected: {ids}')
                    self.stop_rotation()
                    self.aruco_found = True
                    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.05, self.intrinsic_camera, self.distortion)
                    distance = np.linalg.norm(tvecs[0][0])
                    self.get_logger().info(f'Marker distance: {distance:.2f} meters')
                    self.adjust_position(tvecs[0][0])
                    self.cmd_vel_pub.publish(Twist())  # 로봇을 멈추기 위해 모든 속도를 0으로 설정
                    self.processing_thread_running = False  # Set the flag to stop the thread
                    rclpy.shutdown()  # Shutdown ROS
                else:
                    self.get_logger().info('No ArUco marker detected.')
                end_time = time.time()
                self.get_logger().info(f'Image processing took {end_time - start_time:.2f} seconds')

    def adjust_position(self, tvec):
        twist = Twist()

        if abs(tvec[0]) > 0.05:
            twist.linear.y = -0.1 if tvec[0] > 0 else 0.1
        if abs(tvec[1]) > 0.05:
            twist.linear.x = -0.1 if tvec[1] > 0 else 0.1
        if abs(tvec[2] - 0.2) > 0.05:
            twist.linear.z = 0.1 if tvec[2] > 0.2 else -0.1

        if not (twist.linear.x or twist.linear.y or twist.linear.z):
            self.get_logger().info('Position adjusted successfully')
            self.cmd_vel_pub.publish(Twist())  # 로봇을 멈추기 위해 모든 속도를 0으로 설정
            self.reset_state()
            return

        self.cmd_vel_pub.publish(twist)

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

    def reset_state(self):
        self.target_reached = False
        self.aruco_found = False
        self.rotating = False

    def destroy_node(self):
        self.processing_thread_running = False
        self.processing_thread.join()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    nav_service = NavService()
    rclpy.spin(nav_service)
    nav_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
