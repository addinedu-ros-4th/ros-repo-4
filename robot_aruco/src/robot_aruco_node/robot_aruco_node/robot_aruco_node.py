import rclpy
from rclpy.node import Node
from turtles_service_msgs.srv import ArucoNavigateTo
from geometry_msgs.msg import Point
import cv2
import numpy as np

class RobotArucoNode(Node):

    def __init__(self):
        super().__init__('robot_aruco_node')
        self.declare_parameter('aruco_dict', 'DICT_6X6_250')
        aruco_dict_name = self.get_parameter('aruco_dict').value
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, aruco_dict_name))
        self.aruco_params = cv2.aruco.DetectorParameters()

        self.intrinsic_camera = np.array([
            [500.8251042496629, 0, 297.06186891146444],
            [0, 495.01966287604364, 229.461575892311],
            [0, 0, 1]
        ])
        self.distortion = np.array([0.2411284519030748, -1.04497582377272, -0.027966318140935194, -0.016250782774982115, 1.2957406835430951])

        self.new_position_publisher = self.create_publisher(Point, 'new_position', 10)
        self.srv = self.create_service(ArucoNavigateTo, 'start_aruco_detection', self.start_detection_callback)
        
        self.cap = cv2.VideoCapture(0)  # 로봇의 카메라를 통해 영상 캡처
        self.detecting = False
        self.robot_position = None  # 로봇의 현재 위치를 저장할 변수
        self.desired_distance = 1.25  # 초기값으로 설정, 나중에 서비스 요청에서 설정됨

    def start_detection_callback(self, request, response):
        self.get_logger().info(f'Starting detection for Aruco ID: {request.aruco_id} with desired distance {request.distance}')
        self.robot_position = np.array([request.x, request.y, request.z])  # 로봇의 현재 위치 저장
        self.desired_distance = request.distance  # 서비스 요청에서 받은 원하는 거리 저장
        self.detecting = True
        self.process_images(request.aruco_id)
        response.success = True
        response.message = "Aruco detection started successfully"
        return response

    def process_images(self, target_id):
        while self.detecting:
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().error('Failed to capture image')
                continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

            if ids is not None and target_id in ids:
                index = list(ids.flatten()).index(target_id)
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners[index], 0.055, self.intrinsic_camera, self.distortion)
                
                distance = np.linalg.norm(tvec[0][0])
                self.get_logger().info(f'Detected Aruco marker ID: {target_id}, Distance: {distance:.15f}')

                if self.robot_position is not None:
                    # ArUco 마커의 z 좌표를 사용하여 로봇의 새로운 x 좌표 계산
                    current_distance = tvec[0][0][2]  # ArUco 마커의 z 좌표
                    distance_diff = current_distance - self.desired_distance
                    
                    # 새로운 로봇의 x 좌표 계산
                    new_x_position = self.robot_position[0] + distance_diff
                    
                    # 새로운 목표 위치 설정
                    new_position_msg = Point()
                    new_position_msg.x = new_x_position
                    new_position_msg.y = self.robot_position[1] + tvec[0][0][0]  # 로봇의 y 좌표에 ArUco 마커의 x 좌표를 더함
                    new_position_msg.z = 0.0  # Assuming 2D plane

                    self.new_position_publisher.publish(new_position_msg)
                    self.get_logger().info(f'Published new position: {new_position_msg}')
                else:
                    self.get_logger().error('Robot position not set')

                self.detecting = False  # Stop detection after one successful capture

            # cv2.imshow('Frame', frame)
            # if cv2.waitKey(1) & 0xFF == ord('q'):
            #     break

    def shutdown(self):
        self.cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    robot_aruco_node = RobotArucoNode()
    try:
        rclpy.spin(robot_aruco_node)
    except KeyboardInterrupt:
        pass
    finally:
        robot_aruco_node.shutdown()
        robot_aruco_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
