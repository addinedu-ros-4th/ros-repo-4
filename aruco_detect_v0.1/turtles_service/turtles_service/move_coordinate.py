import socket
import pickle
import struct
import cv2
import numpy as np
import threading
import time
import rclpy
from rclpy.node import Node
from turtles_service_msgs.srv import NavigateTo
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Twist
from tf_transformations import euler_from_quaternion, quaternion_from_euler

TCP_IP = '192.168.0.49'  # 서버 IP 주소
TCP_PORT = 5002       # 서버 포트

class NavService(Node):

    def __init__(self):
        super().__init__('nav_service')
        self.srv = self.create_service(NavigateTo, 'turtle_service', self.service_callback)
        self.action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        self.cmd_vel_pub = self.create_publisher(Twist, '/base_controller/cmd_vel_unstamped', 10)

        self.target_reached = False
        self.aruco_found = False
        self.rotating = False
        self.finding_final_id = False

        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        self.intrinsic_camera = np.array([[933.15867, 0, 657.59],
                                          [0, 933.1586, 400.36993],
                                          [0, 0, 1]])
        self.distortion = np.array([-0.43948, 0.18514, 0, 0])

        self.create_timer(2.0, self.rotation_loop)

        self.image_queue = []
        self.processing_thread = threading.Thread(target=self.process_images)
        self.processing_thread_running = True
        self.processing_thread.start()

        self.tcp_thread = threading.Thread(target=self.receive_images_via_tcp)
        self.tcp_thread.start()

    def receive_images_via_tcp(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((TCP_IP, TCP_PORT))
        data = b""
        payload_size = struct.calcsize("L")
        
        while True:
            while len(data) < payload_size:
                packet = s.recv(4096)
                if not packet: break
                data += packet
            packed_msg_size = data[:payload_size]
            data = data[payload_size:]
            msg_size = struct.unpack("L", packed_msg_size)[0]
            
            while len(data) < msg_size:
                data += s.recv(4096)
            frame_data = data[:msg_size]
            data = data[msg_size:]
            
            frame = pickle.loads(frame_data)
            self.image_queue.append(frame)

    def service_callback(self, request, response):
        if self.target_reached:
            response.success = False
            response.message = "Already navigating to pose"
            return response

        self.get_logger().info('Service called: Navigating to pose...')
        goal_msg = self.create_goal_pose(request.x, request.y, request.z, 0.0, 0.0, 0.0, 1.0)

        self.action_client.wait_for_server()
        self._send_goal_future = self.action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(lambda future: self.goal_response_callback(future, request.aruco_id, request.final_id))

        response.success = True
        response.message = "NavigateToPose action called successfully"
        return response

    def goal_response_callback(self, future, aruco_id, final_id):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(lambda future: self.get_result_callback(future, aruco_id, final_id))

    def get_result_callback(self, future, aruco_id, final_id):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result))
        if result:
            self.target_reached = True
            self.get_logger().info('Target reached. Starting ArUco marker detection.')
            self.rotating = False
            self.finding_final_id = False
            self.aruco_id = aruco_id
            self.final_id = final_id

    def rotation_loop(self):
        if self.rotating and not self.aruco_found and not self.finding_final_id:
            twist = Twist()
            twist.angular.z = 0.5  # Reduce rotation speed for better detection
            self.cmd_vel_pub.publish(twist)
            self.get_logger().info('Rotating...')

    def stop_rotation(self):
        self.get_logger().info('Stopping rotation...')
        self.rotating = False
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

    def process_images(self):
        while self.processing_thread_running:
            if self.image_queue:
                frame = self.image_queue.pop(0)
                start_time = time.time()
                if not self.target_reached:
                    continue

                self.get_logger().info('Processing camera image...')
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

                if ids is not None and len(ids) > 0:
                    self.get_logger().info(f'ArUco marker detected: {ids}')
                    if not self.finding_final_id and self.aruco_id in ids:
                        self.aruco_found = True
                        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.05, self.intrinsic_camera, self.distortion)
                        self.get_logger().info(f'Marker distance: {np.linalg.norm(tvecs[0][0]):.2f} meters')
                        self.align_and_approach_marker(tvecs[0][0], 1.5)
                    elif self.finding_final_id and self.final_id in ids:
                        self.stop_rotation()
                        self.aruco_found = True
                        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.05, self.intrinsic_camera, self.distortion)
                        self.get_logger().info(f'Marker distance: {np.linalg.norm(tvecs[0][0]):.2f} meters')
                        self.align_and_approach_marker(tvecs[0][0], 0.5)
                    else:
                        self.get_logger().info('No matching ArUco marker detected.')
                else:
                    self.get_logger().info('No ArUco marker detected.')
                end_time = time.time()
                self.get_logger().info(f'Image processing took {end_time - start_time:.2f} seconds')

    def align_and_approach_marker(self, tvec, target_distance):
        twist = Twist()
        
        # Align to marker
        if abs(tvec[0]) > 0.05:  # Adjust to center horizontally
            twist.angular.z = -0.1 if tvec[0] > 0 else 0.1
            self.cmd_vel_pub.publish(twist)
            time.sleep(1)  # Allow some time to turn
            self.cmd_vel_pub.publish(Twist())  # Stop turning

        if abs(tvec[1]) > 0.05:  # Adjust to center vertically
            twist.linear.y = -0.1 if tvec[1] > 0 else 0.1
            self.cmd_vel_pub.publish(twist)
            time.sleep(1)  # Allow some time to move
            self.cmd_vel_pub.publish(Twist())  # Stop moving

        # Approach marker until within target distance
        while abs(tvec[2] - target_distance) > 0.1:
            twist = Twist()
            twist.linear.x = 0.1 if tvec[2] > target_distance else -0.1
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.5)  # Allow some time to move
            self.cmd_vel_pub.publish(Twist())  # Stop moving

        if target_distance == 1.5:
            self.aruco_found = False
            self.finding_final_id = True
            self.rotating = True
        else:
            self.stop_rotation()

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
        self.finding_final_id = False

    def destroy_node(self):
        self.processing_thread_running = False
        self.processing_thread.join()
        super().destroy_node()

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
