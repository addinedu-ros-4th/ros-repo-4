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
from geometry_msgs.msg import Point

TCP_IP = '192.168.1.104'
TCP_PORT = 3000

class NavArucoService(Node):

    def __init__(self):
        super().__init__('nav_aruco_service')
        self.detected_count = 0  # 탐지 횟수를 추적하는 변수 추가
        self.detection_active = False  # 탐지 활성화 플래그 추가
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.aruco_params = cv2.aruco.DetectorParameters()

        self.image_queue = []
        self.image_queue_lock = threading.Lock()

        # Use calibration data
        self.intrinsic_camera = np.array([
            [500.8251042496629, 0, 297.06186891146444],
            [0, 495.01966287604364, 229.461575892311],
            [0, 0, 1]
        ])

        self.distortion = np.array([0.2411284519030748, -1.04497582377272, -0.027966318140935194, -0.016250782774982115, 1.2957406835430951])

        # Camera offset in robot coordinates (example values)
        self.camera_offset = np.array([0.03, 0.0, 0.0])  # 3 cm in front of the robot

        self.robot_position = None  # 로봇의 위치 저장할 변수 추가

        # 새로운 좌표 퍼블리셔 추가
        self.new_position_publisher = self.create_publisher(Point, 'new_position', 10)

        self.processing_thread_running = True
        self.processing_thread = threading.Thread(target=self.process_images)
        self.processing_thread.start()

        self.tcp_thread_running = True
        self.tcp_thread = threading.Thread(target=self.receive_images_via_tcp)
        self.tcp_thread.start()

        self.srv = self.create_service(ArucoNavigateTo, 'start_aruco_detection', self.start_detection_callback)

    def receive_images_via_tcp(self):
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.get_logger().info('Attempting to connect to server')
            client_socket.connect((TCP_IP, TCP_PORT))
            self.get_logger().info('Connected to server')

            data = b""
            payload_size = struct.calcsize("L")
            
            while self.tcp_thread_running:
                try:
                    while len(data) < payload_size:
                        packet = client_socket.recv(4096)
                        data += packet

                    if len(data) < payload_size:
                        continue

                    packed_msg_size = data[:payload_size]
                    data = data[payload_size:]
                    msg_size = struct.unpack("L", packed_msg_size)[0]

                    while len(data) < msg_size:
                        packet = client_socket.recv(4096)
                        data += packet

                    frame_data = data[:msg_size]
                    data = data[msg_size:]

                    frame = pickle.loads(frame_data)
                    frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)
                    
                    with self.image_queue_lock:
                        self.image_queue.append(frame)

                except Exception as e:
                    self.get_logger().error(f"Error receiving image: {e}")
                    break
        finally:
            client_socket.close()

    def process_images(self):
        while self.processing_thread_running:
            frame = None
            with self.image_queue_lock:
                if self.image_queue:
                    frame = self.image_queue.pop(0)

            if frame is not None:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

                if self.detection_active and ids is not None and len(ids) > 0 and self.detected_count < 3:
                    self.get_logger().info(f'Detected Aruco markers: {ids.flatten()}')
                    for i in range(len(ids)):
                        rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.055, self.intrinsic_camera, self.distortion)
                        
                        # Adjusting for the camera offset to get the correct position in robot's coordinate system
                        tvec_corrected = tvec[0][0] + self.camera_offset

                        # Calculating the distance to the marker using the norm of the translation vector
                        distance = np.linalg.norm(tvec_corrected)
                        self.get_logger().info(f'Marker ID: {ids[i][0]}, Position: {tvec_corrected}, Distance: {distance:.15f}')
                        
                        # Calculate the new robot position to have the marker 1.25 meters away
                        if self.robot_position is not None:
                            new_map_position = self.calculate_new_map_position(tvec_corrected, 1.1)
                            self.get_logger().info(f'New map position to achieve 1.25m distance: {new_map_position}')
                            
                            # Publish the new position
                            self.publish_new_position(new_map_position)
                    
                    self.detected_count += 1
                    if self.detected_count >= 1:  # 탐지 횟수가 3회 이상일 경우 추가 처리(예: 탐지 중단)
                        self.get_logger().info('3 Aruco markers detected, stopping detection.')
                        self.detection_active = False

                cv2.imshow('Frame Video', frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            time.sleep(0.1)

    def publish_new_position(self, new_position):
        new_position_msg = Point()
        new_position_msg.x = new_position[0]
        new_position_msg.y = new_position[1]
        new_position_msg.z = 0.0

        self.new_position_publisher.publish(new_position_msg)
        self.get_logger().info(f'Published new position: {new_position}')

    def calculate_new_map_position(self, aruco_position, desired_distance):
        # 현재 ArUco 마커와의 거리
        current_distance = aruco_position[2]  # Z 좌표가 거리

        # 목표 거리와의 차이
        distance_diff = current_distance - desired_distance

        # 새로운 로봇의 X 좌표 계산
        new_x_position = self.robot_position[0] + distance_diff

        # 새로운 목표 위치 계산 (맵 좌표계에서 로봇의 Y, Z 좌표는 그대로 사용)
        new_position = [new_x_position, self.robot_position[1], self.robot_position[2]]
        return np.round(new_position, 15)  # 소수점 15자리까지 반올림

    def start_detection_callback(self, request, response):
        self.get_logger().info(f'Starting detection for Aruco ID: {request.aruco_id}')
        self.detected_count = 0  # 탐지 시작 시 탐지 횟수를 0으로 초기화
        self.detection_active = False  # 탐지 비활성화
        self.robot_position = [request.x, request.y, request.z]
        self.get_logger().info(f'Robot position set to: {self.robot_position}')  # 디버깅 메시지 추가
        # 탐지 활성화는 로봇이 도착한 후에 수행
        time.sleep(5)  # 로봇이 도착할 시간을 기다림 (예시: 5초 대기)
        self.detection_active = True
        response.success = True
        response.message = "Aruco detection started successfully after reaching the destination"
        return response

    def shutdown(self):
        self.processing_thread_running = False
        self.processing_thread.join()
        self.tcp_thread_running = False
        self.tcp_thread.join()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    nav_aruco_service = NavArucoService()
    try:
        rclpy.spin(nav_aruco_service)
    except KeyboardInterrupt:
        pass
    finally:
        nav_aruco_service.shutdown()
        nav_aruco_service.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
