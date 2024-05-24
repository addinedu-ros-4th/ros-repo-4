import socket
import pickle
import struct
import cv2
import numpy as np
import threading
import time
import rclpy
from rclpy.node import Node
from turtles_service_msgs.srv import ArucoNavigateTo
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Twist
from tf_transformations import euler_from_quaternion, quaternion_from_euler

TCP_IP = '192.168.0.20'  # 클라이언트 IP 주소
TCP_PORT = 5002          # 서버 포트

class Nav_Aruco_Service(Node):

    def __init__(self):
        super().__init__('Nav_Aruco_Service')
        self.srv = self.create_service(ArucoNavigateTo, 'correct_turtles', self.service_callback)
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

        self.create_timer(0.1, self.rotation_loop)

        self.image_queue = []
        self.processing_thread = threading.Thread(target=self.process_images)
        self.processing_thread_running = True
        self.processing_thread.start()

        self.tcp_thread = threading.Thread(target=self.receive_images_via_tcp)
        self.tcp_thread.start()

    def receive_images_via_tcp(self):
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.bind((TCP_IP, TCP_PORT))
        server_socket.listen(1)
        self.get_logger().info('Waiting for connection...')
        conn, addr = server_socket.accept()
        self.get_logger().info(f'Connected by {addr}')

        data = b""
        payload_size = struct.calcsize("L")
        
        while True:
            while len(data) < payload_size:
                packet = conn.recv(4096)
                if not packet: break
                data += packet
            packed_msg_size = data[:payload_size]
            data = data[payload_size:]
            msg_size = struct.unpack("L", packed_msg_size)[0]
            
            while len(data) < msg_size:
                data += conn.recv(4096)
            frame_data = data[:msg_size]
            data = data[msg_size:]
            
            frame = pickle.loads(frame_data)
            self.image_queue.append(frame)

    def service_callback(self, request: ArucoNavigateTo.Request, response: ArucoNavigateTo.Response):

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
            self.aruco_found = False

    def rotation_loop(self):
        if self.finding_final_id and not self.aruco_found:
            twist = Twist()
            twist.angular.z = 0.15  # 작은 각도 회전
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.2)  # 회전 시간 단축
            self.cmd_vel_pub.publish(Twist())  # 회전 멈춤
            # 여기서 탐지를 호출하지 않고, `process_images` 스레드가 계속 탐지하도록 함

    def stop_rotation(self):
        self.get_logger().info('Stopping rotation...')
        self.rotating = False
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

    def process_images(self):
        while self.processing_thread_running:
            if self.image_queue:
                frame = self.image_queue.pop(0)
                if not self.target_reached and not self.finding_final_id:
                    continue

                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

                if ids is not None and len(ids) > 0:
                    if not self.finding_final_id and self.aruco_id in ids:
                        self.aruco_found = True
                        _, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.05, self.intrinsic_camera, self.distortion)
                        self.align_and_approach_marker(tvecs[0][0], 1.5)
                    elif self.finding_final_id and self.final_id in ids:
                        self.aruco_found = True
                        _, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.05, self.intrinsic_camera, self.distortion)
                        self.align_and_approach_marker(tvecs[0][0], 0.5, True)
            # 탐지 후 바로 다음 이미지를 처리할 수 있도록 대기 시간을 줄임
            time.sleep(0.1)

    def align_and_approach_marker(self, tvec, target_distance, is_final=False):
        while tvec is not None and abs(tvec[2] - target_distance) > 0.1:
            twist = Twist()

            if not is_final:
                # Only move forward or backward to reach the target distance
                twist.linear.x = 0.1 if tvec[2] > target_distance else -0.1
                self.cmd_vel_pub.publish(twist)
                time.sleep(0.5)  # Allow some time to move
                self.cmd_vel_pub.publish(Twist())  # Stop moving
                tvec = self.check_marker_position(final_check=is_final)

        self.cmd_vel_pub.publish(Twist())  # Final stop to ensure the robot has stopped

        if target_distance == 1.5:
            self.aruco_found = False
            self.finding_final_id = True
        elif is_final and target_distance == 0.5:
            self.get_logger().info('Reached final target distance')

    def check_marker_position(self, final_check=False):
        while self.image_queue:
            frame = self.image_queue.pop(0)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
            if ids is not None and len(ids) > 0:
                if final_check and self.final_id in ids:
                    _, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.05, self.intrinsic_camera, self.distortion)
                    return tvecs[0][0]
                elif not final_check and self.aruco_id in ids:
                    _, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.05, self.intrinsic_camera, self.distortion)
                    return tvecs[0][0]
        return None

    def check_for_final_marker(self):
        position = self.check_marker_position(final_check=True)
        if position is not None:
            self.aruco_found = True
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
    nav_service = Nav_Aruco_Service()
    try:
        rclpy.spin(nav_service)
    except KeyboardInterrupt:
        pass
    nav_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
