import sys
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5 import uic, QtCore
from PyQt5.QtCore import QThread, pyqtSignal,Qt,QDate
from PyQt5 import QtWidgets, QtCore
import rclpy as rp
from rclpy.node import Node
from geometry_msgs.msg import Twist
# from turtles_service_msgs.srv import NavToPose
import time
import socket
import select 
import struct 
import pickle
import cv2
import pandas as pd 
from enum import Enum
from YamlFileManager import YamlFileManager
from DBManager import DBManager
from datetime import datetime, timedelta
import torch
import torchvision.transforms as transforms
import numpy as np 
import matplotlib.pyplot as plt
#encryption
from Crypto.Cipher import AES
from Crypto.Random import get_random_bytes
from PyQt5.QtCore import Qt, QRect
rp.init()
import os
import cv2
from ultralytics import YOLO
import atexit

    
class Pages(Enum):
    #page 상수 정의
    PAGE_HOME = 0
    PAGE_LOGIN = 1
    PAGE_MONITOR_BARN = 2
    PAGE_MONITOR_FACILITIES = 3 
    PAGE_MONITOR_CAMERA = 4
    PAGE_CHOOSE_CONTROL_ROBOT = 5
    PAGE_CONTROL_ROBOT = 6
    PAGE_CONTROL_FACILITIES = 7 
    PAGE_CHOOSE_ROBOTMANAGER_TASK = 8
    PAGE_ROBOTMANAGER_TASK = 9
    PAGE_CHOOSE_DATAMANAGER_ANIMAL = 10
    PAGE_REGISTER_ANIMAL = 11
    PAGE_SEARCH_ANIMAL = 12
    PAGE_CHOOSE_DATAMANAGER_FOOD = 13
    PAGE_REGISTER_FOOD = 14
    PAGE_SEARCH_FOOD = 15
    PAGE_DATAMANAGER_VIDEO = 16
    PAGE_CHOOSE_DATAMANAGER_FACILITIES = 17
    PAGE_REGISTER_EMPLOYEE = 18
    PAGE_REGISTER_OTHERS = 19
    PAGE_SCHEDULE_FOOD = 20
    PAGE_SCHEDULE_FACILITIES = 21
    PAGE_LOG = 22
    PAGE_SETTING = 23
    

class TaskScheduleType(Enum):
    TASK_SCHEDULED = 0  # schedule 화면에서 저장된 업무
    TASK_REGISTERED = 1 # Robot Manager Task 화면에서 등록한 업무 

class TaskType(Enum):
    TASK_FOOD = 1
    TASK_CLEANING = 2
    TASK_ARRANGE = 3

#TaskType이랑 RobotType은 숫자 같게 맞춰줘야 함... 일단은..
class RobotType(Enum):
    ROBOT_FOOD = 1
    ROBOT_CLEANING = 2
    ROBOT_ARRANGE = 3

class Status(Enum):
    STATUS_STANDBY = 0
    STATUS_NAV_ARUCO = 1
    STATUS_FOOD_CHARGE = 2
    STATUS_MANUAL_MOVE= 3
    STATUS_FOOD_DISTRIBUTE = 4
    STATUS_RETURN = 5



class RobotStatus:
    def __init__(self,robot_num, type):
        self.robot_num = robot_num
        self.robot_type = type # 1: food_robot 2: cleaning_robot 3: arrange_robot
        self.status = Status.STATUS_STANDBY.value
        self.task_id = 0

    def setStatus(self,robot_status):
        self.status = robot_status

    def setTaskID(self,task_id):
        self.task_id = task_id

    def getTaskID(self):
        return self.task_id

class Task:
    def __init__(self,id,schedule_type, room_num,task_time):
        self.task_id = id 
        self.task_type = TaskType.TASK_FOOD.value   #0: 배식 1:청소 2: 정리
        self.task_schedule_type = schedule_type     #0: schedule, 1: registered
        self.assigned_robot_num = 0                 #배정된 로봇 이름 
        self.task_result = 0                        #0: not done , 1: done
        self.task_time = task_time                  #'2024-01-01 00:00:00' # 업무 할당 시간
        self.task_room_num = room_num               # room은 1부터 시작 1,2,3,4
        self.task_current_progress = 0              # 1: 업무 할당됨, 2: 사료 탱크 도착 3: 사료 받기 완료 4: 축사앞 도착 완료 5: 먹이통 도착 완료 6: 먹이 주기 완료  
        self.task_food_tank_num = 0                 
    
    def setTaskResult(self, task_result):
        self.task_result = task_result

    def setFoodTank(self, num):
        self.task_food_tank_num = num
    
    def updateTaskProgress(self):
        self.task_current_progress = self.task_current_progress +1 

    def getCurrentProgress(self):
        return self.task_current_progress
    
    def getFoodTank(self):
        return self.task_food_tank_num

class Teleop(Node):
    def __init__(self):
        super().__init__('move_up')
        self.publisher = self.create_publisher(Twist, '/base_controller/cmd_vel_unstamped', 10)
        self.twist = Twist()
        

    def publish_twist(self):
        self.publisher.publish(self.twist)

class RosThread(QThread):

    new_connection = pyqtSignal(object)

    def __init__(self,ros_node):
        super().__init__()
        self.node = ros_node
        self.new_connection.emit(self.node)

    def run(self):
        self.node.publish_twist()
        self.msleep(100) #추가 안하면 UI에 딜레이 발생
        rp.spin(self.node)
            
    
class Camera(QThread):
    update = QtCore.pyqtSignal()

    def __init__(self, sec =0, parent = None):
        super().__init__()
        self.main = parent
        self.running = True

    def run(self):
        while self.running == True:
            self.update.emit()
            time.sleep(0.1)

    def stop(self):
        self.running = False
    
class RobotThread(QThread):
    update = QtCore.pyqtSignal()

    def __init__(self, sec =0, parent = None):
        super().__init__()
        self.main = parent
        self.moving = True

    def run(self):
        while self.moving == True:
            self.update.emit()
            time.sleep(0.5)
    
    def stop(self):
        self.moving = False
    

class ServerThread(QThread):
    new_connection = pyqtSignal(str, int)
    connection_lost = pyqtSignal(str, int)
    image_signal = pyqtSignal(QImage)
    client_socket_list = []

    def __init__(self, host, port):
        super().__init__()
        self.host = host
        self.port = port
        self.server_socket = None
        self.client_sockets = []

    def run(self):
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen(5)
        self.client_sockets.append(self.server_socket)
        ServerThread.client_socket_list=self.client_sockets
        print(f"서버가 {self.host}:{self.port}에서 대기 중입니다...")

        while True:
            readable, _, _ = select.select(self.client_sockets, [], [])

            for sock in readable:
                if sock == self.server_socket:
                    # 새로운 클라이언트 연결
                    client_socket, client_address = self.server_socket.accept()
                    self.client_sockets.append(client_socket)
                    print(f"클라이언트 {client_address}가 연결되었습니다.")
                    self.new_connection.emit(client_address[0], client_address[1])
                    data = b""
                    payload_size = struct.calcsize("L")

                    while True:
                        while len(data) < payload_size:
                            packet = client_socket.recv(4096)
                            if not packet:
                                return
                            data += packet

                        packed_msg_size = data[:payload_size]
                        data = data[payload_size:]
                        msg_size = struct.unpack("L", packed_msg_size)[0]

                        while len(data) < msg_size:
                            data += client_socket.recv(4096)

                        frame_data = data[:msg_size]
                        data = data[msg_size:]

                        frame = pickle.loads(frame_data)
                        q_img = self.cv2_to_qimage(frame)
                        self.image_signal.emit(q_img)
                        
                        if data == None:
                            # 클라이언트 연결 종료
                            client_address = sock.getpeername()
                            print(f"클라이언트 {client_address} 연결이 끊어졌습니다.")
                            self.client_sockets.remove(sock)
                            sock.close()
                            self.connection_lost.emit(client_address[0], client_address[1])
                            break                        
                else:
                    # 기존 클라이언트의 데이터 수신 및 처리
                    data = b""
                    payload_size = struct.calcsize("L")

                    while True:
                        while len(data) < payload_size:
                            packet = client_socket.recv(4096)
                            if not packet:
                                return
                            data += packet

                        packed_msg_size = data[:payload_size]
                        data = data[payload_size:]
                        msg_size = struct.unpack("L", packed_msg_size)[0]

                        while len(data) < msg_size:
                            data += client_socket.recv(4096)

                        frame_data = data[:msg_size]
                        data = data[msg_size:]

                        frame = pickle.loads(frame_data)
                        q_img = self.cv2_to_qimage(frame)
                        self.image_signal.emit(q_img)
                        
                        if data == None:
                            # 클라이언트 연결 종료
                            client_address = sock.getpeername()
                            print(f"클라이언트 {client_address} 연결이 끊어졌습니다.")
                            self.client_sockets.remove(sock)
                            sock.close()
                            self.connection_lost.emit(client_address[0], client_address[1])
                            break
                            
    def cv2_to_qimage(self, cv2_image):
        height, width, channel = cv2_image.shape
        bytes_per_line = 3 * width
        # BGR에서 RGB로 변환
        rgb_image = cv2.cvtColor(cv2_image, cv2.COLOR_BGR2RGB)
        q_img = QImage(rgb_image.data, width, height, bytes_per_line, QImage.Format_RGB888)
        return q_img
    
    def stop(self):
        for sock in self.client_sockets:
            sock.close()
            self.server_socket.close()
            print("서버를 종료합니다.")
               
from_class = uic.loadUiType("Turtles.ui")[0] 

class Camera(QThread):
    update = pyqtSignal()
    
    def __init__(self, parent=None):
        super().__init__()
        self.main = parent
        self.running = True
    
    def run(self):
        while self.running:
            self.update.emit()
            time.sleep(0.1)
    
    def stop(self):
        self.running = False

class WindowClass(QMainWindow, from_class) :
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.setWindowTitle("Turtles : Herding Heroes")

        # Camera check
        self.available_index = []
        self.camera_list = []

        for index in range(15): 
            camera = cv2.VideoCapture(index)
            if camera.isOpened():
                self.available_index.append(index)
                camera.release()
        if len(self.available_index)> 0:
            for val in self.available_index:
                self.temp_cap = cv2.VideoCapture(val)
                self.camera_list.append(self.temp_cap)

        for idx,camera in enumerate(self.camera_list):
            if camera.isOpened():
                camera.set(cv2.CAP_PROP_FPS, 30)
            else:
                print("camera index", end="")
                print(idx)
                print("is not opened")

        print(self.available_index)
        
        self.camera = Camera(self)
        self.camera.daemon = True
        self.pixmap = QPixmap()

        self.camera.update.connect(self.updateCameraView)

        self.select_camera_box.currentIndexChanged.connect(self.combochanged)
        self.cam_num = 0
        self.cameraStart()

        self.isCameraOn = False
        self.isRecStart = False
        
        self.record = Camera(self)
        self.record.daemon = True

        self.btnRecord.clicked.connect(self.clickRecord)
        self.record.update.connect(self.updateRecording)
        self.count = 0
        self.btnCapture.clicked.connect(self.capture)   #     

        #robot task 리스트
        self.task_list = []
        self.yaml_file = YamlFileManager('config.yaml')
        self.task_id = self.yaml_file.getLastTaskID()
        self.client_module = self.yaml_file.getClientMoudule()
        self.robot_list = []
        food_robot1 = RobotStatus(1,RobotType.ROBOT_FOOD.value)
        # food_robot2 = RobotStatus(2,RobotType.ROBOT_FOOD.value)
        
        self.robot_list.append(food_robot1)
        # self.robot_list.append(food_robot2)

        # yolo 
        self.yolo_detect_class = []
        self.yolo_detect_class_coordinate = []
        self.yolo_harmful_animal_detect_class = []
        self.yolo_harmful_animal_detect_class_coordinate = []
        # Load the YOLOv8 model
        self.model = YOLO('yolov8n.pt')


        #databases 연결
        self.data_manage = DBManager("192.168.1.101", "0000", 3306, "turtles", "TurtlesDB")
        self.animal_df = self.data_manage.getAnimal()
        self.camera_df = self.data_manage.getCameraPath()
        self.food_df = self.data_manage.getFood()
        self.schedule_df= self.data_manage.getFoodRobotSchedule()
        self.userdata_df = self.data_manage.getUserData()
        self.employee_df = self.data_manage.getEmployeeData()
        self.harmful_animal_df = self.data_manage.getHarmfulAnimal()
        self.facility_setting_df = self.data_manage.getFacilitySetting()
        self.robot_thread = RobotThread(parent=self)
        self.count = 0
        self.server_thread = None
        self.client_df = pd.DataFrame(columns=['IP', 'Port'])
        self.robotThreadStart()

        self.quit_button.hide()
        self.server_label.hide()
        self.client_label.hide()
        self.connect_button.clicked.connect(self.start_tcp_server_thread)
        self.quit_button.clicked.connect(self.stop_tcp_server_thread)
        self.client_table.horizontalHeader().setStretchLastSection(True)
        self.client_table.setColumnWidth(0, 280)
         
        #login 화면으로 초기화면 셋팅
        self.stackedWidget.setCurrentIndex(Pages.PAGE_LOGIN.value)
        self.toolBox.setCurrentIndex(7)

        #버튼과 페이지 이동 연결
        self.home_page_button.clicked.connect(self.home_page_button_clicked)
        
        self.monitor_barnpage_button.clicked.connect(self.monitor_barnpage_button_clicked)
        self.monitor_facilitiespage_button.clicked.connect(self.monitor_facilitiespage_button_clicked)
        self.monitor_camerapage_button.clicked.connect(self.monitor_camerapage_button_clicked) #

        self.control_robotpage_button.clicked.connect(self.control_robotpage_button_clicked) 
        self.control_robot_a_button.clicked.connect(lambda: self.control_robotpage("Food Robot A"))
        self.control_robot_b_button.clicked.connect(lambda: self.control_robotpage("Food Robot B"))
        
        self.control_facilitiespage_button.clicked.connect(self.control_facilitiespage_button_clicked)
        
        self.robotmanager_taskpage_button.clicked.connect(self.robotmanager_choose_button_clicked)
        self.manage_robot_a_button.clicked.connect(lambda: self.robotmanager_taskpage_button_clicked("Food Robot A"))
        self.manage_robot_b_button.clicked.connect(lambda: self.robotmanager_taskpage_button_clicked("Food Robot B"))

        self.datamanager_animalpage_button.clicked.connect(self.choose_datamanager_animalpage_button_clicked)
        self.choose_register_button_animal.clicked.connect(self.register_animal_button_clicked)
        self.choose_search_button_animal.clicked.connect(self.search_animal_button_clicked)
        
        self.datamanager_foodpage_button.clicked.connect(self.choose_datamanager_foodpage_button_clicked)
        self.choose_register_button_food.clicked.connect(self.register_food_button_clicked)
        
        self.choose_search_button_food.clicked.connect(self.search_food_button_clicked)
        
        self.datamanager_videopage_button.clicked.connect(self.datamanager_videopage_button_clicked)
        
        self.datamanager_facilitiespage_button.clicked.connect(self.choose_datamanager_facilitiespage_button_clicked)
        self.choose_register_employee_button.clicked.connect(self.register_employee_button_clicked)
        self.choose_others_button.clicked.connect(self.register_others_button_clicked)

        self.schedule_foodpage_button.clicked.connect(self.schedule_foodpage_button_clicked)
        self.schedule_facilitiespage_button.clicked.connect(self.schedule_facilitiespage_button_clicked)

        #self.toolBox.currentChanged.connect(self.toolbox_changed) ## 버튼 페이지 연결 
        self.setting_button.clicked.connect(self.setting_page_button_clicked)
        self.log_button.clicked.connect(self.log_page_button_clicked)

        #login logout 버튼 연결
        self.logout_button.clicked.connect(self.logout_button_clicked)
        self.login_button.clicked.connect(self.login_button_clicked)

    
        #robot thread  
        self.robot_thread.update.connect(self.update_robot_thread)

        # teleop_button connect
        self.up_button.clicked.connect(self.foodrobot_up_button_clicked)
        self.down_button.clicked.connect(self.foodrobot_down_button_clicked)
        self.left_button.clicked.connect(self.foodrobot_left_button_clicked)
        self.right_button.clicked.connect(self.foodrobot_right_button_clicked)
        #food trailer servo 버튼 연결
        self.foodtank_servo_open_button.clicked.connect(self.foodtank_servo_open_button_clicked)
        self.foodtank_servo_close_button.clicked.connect(self.foodtank_servo_close_button_clicked)
        self.foodtrailer_servo_open_button.clicked.connect(self.foodtrailer_servo_open_button_clicked)
        self.foodtrailer_servo_close_button.clicked.connect(self.foodtrailer_servo_close_button_clicked)

        #task add
        self.task_add_button.clicked.connect(self.task_add_button_clicked)

        #ros 
        self.service_client_node = rp.create_node('client_test')

        self.layout = QVBoxLayout()
        self.setTableWidget(self.feeding_table) 
        self.layout.addWidget(self.feeding_table)
        self.setTableWidget(self.ventilation_table)
        self.layout.addWidget(self.ventilation_table)
        self.setLayout(self.layout) 
        
        

        if torch.cuda.is_available():
            print("GPU 사용가능 여부를 확인중입니다......")
        # GPU를 사용하도록 설정
            self.device = torch.device("cuda")
            print("GPU를 사용합니다.")
        else:
        # CPU를 사용하도록 설정
            self.device = torch.device("cpu")
            print("GPU를 사용할 수 없습니다. CPU를 사용합니다.")
            
        current_date = QDate.currentDate().toString('yyyy-MM-dd')
        #등록일에 현재 날짜 설정
        self.registered_date_animal.setText(current_date); self.registered_date_animal.setReadOnly(True)
        self.registered_date_food.setText(current_date); self.registered_date_food.setReadOnly(True)
        self.registered_date_employee.setText(current_date); self.registered_date_employee.setReadOnly(True)
        self.login_information_label.setText("")
        
        #테이블에 db 정보 불러오기 
        self.animal_df.rename(columns={'age': 'age (in months)', 'weight': 'weight (kg)'}, inplace=True)
        self.load_data_to_table(self.search_animal_table, self.animal_df)
        
        self.food_df.rename(columns={'weight': 'weight (kg)'}, inplace=True)
        self.load_data_to_table(self.search_food_table, self.food_df)
        
        self.camera_df.rename(columns={'camera_num': 'cam_num'}, inplace=True)
        self.load_data_to_table(self.search_camera_table, self.camera_df)
        self.load_data_to_table(self.registered_employee_table, self.employee_df)
        self.load_data_to_table(self.harmful_animal_table, self.harmful_animal_df)
        self.load_data_to_table(self.facility_table,self.facility_setting_df)
        
        self.auto_resize_columns(self.search_animal_table)
        self.auto_resize_columns(self.search_food_table)
        self.auto_resize_columns(self.search_camera_table)
        self.auto_resize_columns(self.registered_employee_table)
        self.auto_resize_columns(self.harmful_animal_table)
        self.auto_resize_columns(self.facility_table)
        
        
        # animal_df 컬럼 데이터를 ComboBox에 추가
        self.load_combobox(self.animal_df, 'animal_id', self.search_id_box)
        self.load_combobox(self.animal_df, 'gender', self.search_gender_box)
        self.load_combobox(self.animal_df, 'age (in months)', self.search_age_box)
        self.load_combobox(self.animal_df, 'food_brand', self.search_food_brand_box)
        self.load_combobox(self.animal_df, 'room', self.search_room_box)
        self.load_combobox(self.animal_df, 'weight (kg)', self.search_weight_box)
        self.load_combobox(self.animal_df, 'registered_date', self.search_rfid_box)
        self.load_combobox(self.animal_df, 'rfid_uid', self.search_animal_date_box)
        
        # food_df 컬럼 데이터를 ComboBox에 추가
        self.load_combobox(self.food_df, 'barcode_id', self.search_food_id_box)
        self.load_combobox(self.food_df, 'brand_name', self.search_brand_name_box)
        self.load_combobox_except_all(self.food_df, 'brand_name', self.select_feed_box)
        self.load_combobox(self.food_df, 'weight (kg)', self.search_food_weight_box)
        self.load_combobox(self.food_df, 'registered_date', self.search_registered_date_box)
        self.load_combobox(self.food_df, 'expiry_date', self.search_expiry_date_box)
        
        
        #camera_df 컬럼 데이터 
        self.load_combobox(self.camera_df, 'cam_num', self.select_camera_box_video)
        self.load_combobox(self.camera_df, 'info_type', self.select_camera_type_box)
        self.load_combobox(self.camera_df, 'captured_date', self.captured_date_start)
        self.load_combobox(self.camera_df, 'captured_date', self.captured_date_end)
        
        self.search_button_animal.clicked.connect(self.animal_search)
        self.search_button_food.clicked.connect(self.food_search)
        # 시작 날짜 콤보박스 변경 시 종료 날짜 콤보박스 업데이트
        self.captured_date_start.currentIndexChanged.connect(self.update_captured_date_end)
        self.search_button_video.clicked.connect(self.camera_search)
        self.employee_register_button.clicked.connect(self.register_new_employee)
        self.add_harmful_animal_button.clicked.connect(self.register_new_harmful_animal)
        #self.delete_harmful_animal_button.clicked.connect(self.delete_harmful_animal)
        self.refresh_combo_box()
        self.register_button_food.clicked.connect(self.register_new_food)
        self.register_button_animal.clicked.connect(self.register_new_animal)
        self.update_button.clicked.connect(self.update_facility_setting)
        

        
        self.set_food_button.clicked.connect(self.on_set_food_button_click)
        self.populate_table()
        self.set_room_box.currentIndexChanged.connect(self.populate_table)
        self.checked_times = [] 
        self.set_facility_schedule_button.clicked.connect(self.setFacilitySchedule)
        self.display_existing_schedule()
        
        self.is_logged_in = False  # 로그인 상태 변수
        self.buttons = [
            self.monitor_barnpage_button, self.monitor_camerapage_button, self.monitor_facilitiespage_button,
            self.control_facilitiespage_button, self.control_robotpage_button, self.robotmanager_taskpage_button,
            self.datamanager_animalpage_button, self.datamanager_facilitiespage_button, self.datamanager_foodpage_button,
            self.datamanager_videopage_button, self.schedule_facilitiespage_button, self.schedule_foodpage_button,
            self.home_page_button, self.notification_page_button, self.shortcut_page_button, self.log_button, self.setting_button
        ]
        for button in self.buttons:
            button.clicked.connect(self.handle_button_click)
        self.update_buttons()
        self.permission=''
        self.UserEdit.hide()
        self.shortcut_page_button.clicked.connect(self.show_shortcut_dialog)
        self.current_active_button = None
        self.notifications = []  # 알림 저장 리스트
        self.notification_page_button.clicked.connect(self.show_notification_dialog)
        self.setting_button.setEnabled(False)
        
        self.intake_df=self.data_manage.getFoodIntake()
        self.pose_df = self.data_manage.getAnimalPose()
        self.sensor_df = self.data_manage.getSensorData()
        self.schedule_num=self.data_manage.getSchedulenums()        
        self.displayFoodIntake()
        self.displayanimalPose()
        self.displaySensor()
        
        self.feedingtime_display_label.setText(str(self.schedule_num)) # 
        self.record_label.hide()
        

        
    def displayFoodIntake(self):
        fig, ax = plt.subplots(figsize=(5, 3))
        self.intake_df['Average'] = self.intake_df[['Room1', 'Room2', 'Room3', 'Room4']].mean(axis=1)
        # 각 방별 섭취량을 꺾은선 그래프로 그리기 (색상 변경)
        ax.plot(self.intake_df['Date'], self.intake_df['Room1'], marker='o', label='Room 1', color='red')
        ax.plot(self.intake_df['Date'], self.intake_df['Room2'], marker='o', label='Room 2', color='green')
        ax.plot(self.intake_df['Date'], self.intake_df['Room3'], marker='o', label='Room 3', color='blue')
        ax.plot(self.intake_df['Date'], self.intake_df['Room4'], marker='o', label='Room 4', color='cyan')
        ax.plot(self.intake_df['Date'], self.intake_df['Average'], marker='o', linestyle='--', color='black', label='Average')
        # 그래프 설정
        ax.set_title('Food Intake by Room and Date')
        ax.set_xlabel('Date');ax.set_ylabel('Food Intake')
        ax.legend();ax.grid(True)
        fig.tight_layout()
        # 이미지를 QPixmap으로 변환하여 QLabel에 삽입
        canvas = fig.canvas
        canvas.draw()
        # RGB 이미지로 변환
        width, height = canvas.get_width_height()
        image = QImage(canvas.tostring_rgb(), width, height, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(image)
        self.food_intake_average_label.setPixmap(pixmap)
        label_size = self.food_intake_average_label.size()
        self.food_intake_average_label.setFixedSize(label_size)
        self.drawTank(self.water_tank_display_label, 70)
        self.drawTank(self.food_tank_A_display_label, 50)
        self.drawTank(self.food_tank_B_display_label, 30)

    def drawTank(self, label, percentage):
        # Create a QPixmap with the same size as the label
        pixmap = QPixmap(label.size())
        pixmap.fill(Qt.transparent)  # Fill with transparent background

        painter = QPainter(pixmap)
        
        # Draw the border
        rect = label.rect()
        painter.drawRect(rect)

        # Fill the percentage area
        fill_height = int(rect.height() * (percentage / 100.0))
        fill_rect = QRect(rect.x(), rect.y() + rect.height() - fill_height, rect.width(), fill_height)
        painter.fillRect(fill_rect, QColor(100, 150, 255))

        # Draw the percentage text
        painter.setFont(QFont('Arial', 14, QFont.Bold))
        painter.setPen(QColor(0, 0, 0))
        painter.drawText(rect, Qt.AlignCenter, f'{percentage}%')
        
        painter.end()
        
        # Set the pixmap to the label
        label.setPixmap(pixmap)

    
    def displaySensor(self):
        fig, ax = plt.subplots(figsize=(6, 2.5))
        # 각 방별 섭취량을 꺾은선 그래프로 그리기 (색상 변경)
        ax.plot(self.sensor_df['time'], self.sensor_df['Temperature'], marker='o', label='Temperature(°C)', color='red')
        ax.plot(self.sensor_df['time'], self.sensor_df['Humidity'], marker='o', label='Humidity(%)', color='green')
        ax.plot(self.sensor_df['time'], self.sensor_df['Luminance'], marker='o', label='Luminance(%)', color='blue')
       # 그래프 설정
        ax.set_title('IoT Data')
        ax.set_xlabel('Time')
        ax.set_ylabel('Sensor data')
        ax.legend(fontsize='small')
        ax.grid(True)
        fig.tight_layout()
        # 이미지를 QPixmap으로 변환하여 QLabel에 삽입
        canvas = fig.canvas
        canvas.draw()
        # RGB 이미지로 변환
        width, height = canvas.get_width_height()
        image = QImage(canvas.tostring_rgb(), width, height, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(image)
        self.iot_display_label.setPixmap(pixmap)
        label_size = self.iot_display_label.size()
        self.iot_display_label.setFixedSize(label_size)  

  
    def displayanimalPose(self):   
        fig, ax = plt.subplots(figsize=(5, 2.5))
        self.pose_df['Average'] = self.pose_df[['Room1', 'Room2', 'Room3', 'Room4']].mean(axis=1)
        # 각 방별 섭취량을 꺾은선 그래프로 그리기 (색상 변경)
        ax.plot(self.pose_df['time'], self.pose_df['Room1'], marker='o', label='Room 1', color='red')
        ax.plot(self.pose_df['time'], self.pose_df['Room2'], marker='o', label='Room 2', color='green')
        ax.plot(self.pose_df['time'], self.pose_df['Room3'], marker='o', label='Room 3', color='blue')
        ax.plot(self.pose_df['time'], self.pose_df['Room4'], marker='o', label='Room 4', color='cyan')
        ax.plot(self.pose_df['time'], self.pose_df['Average'], marker='o', linestyle='--', color='black', label='Average')
        # 그래프 설정
        ax.set_title('Animal pose by Room and Time')
        ax.set_xlabel('Time')
        ax.set_ylabel('Animal pose')
        ax.legend(fontsize='small')
        ax.grid(True)
        fig.tight_layout()
        # 이미지를 QPixmap으로 변환하여 QLabel에 삽입
        canvas = fig.canvas
        canvas.draw()
        # RGB 이미지로 변환
        width, height = canvas.get_width_height()
        image = QImage(canvas.tostring_rgb(), width, height, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(image)
        self.animal_posture_average_label.setPixmap(pixmap)
        label_size = self.animal_posture_average_label.size()
        self.animal_posture_average_label.setFixedSize(label_size)   
        
    
        
        
    def show_notification_dialog(self):
        dialog = QDialog(self)
        dialog.setWindowTitle('Notifications')
        dialog.resize(400, 500)  # 팝업 크기 조정
        
        layout = QVBoxLayout()
        
        notification_text = QTextEdit()
        notification_text.setReadOnly(True)
        
        if len(self.notifications) != 0:
            notification_text.setText('\n'.join(self.notifications))
        else:
            notification_text.setText('No notifications.')
        
        layout.addWidget(notification_text)
        
        dialog.setLayout(layout)
        dialog.exec_()
        
    def handle_button_click(self):
        clicked_button = self.sender()
        if self.current_active_button:
            self.current_active_button.setStyleSheet("font-size: 20px; font-weight: bold;")  # 이전 버튼의 색상 초기화
        
        clicked_button.setStyleSheet("font-size: 20px;font-weight: bold; background-color: darkgray")  # 현재 버튼의 색상 설정
        self.current_active_button = clicked_button
        
    def show_shortcut_dialog(self):
        dialog = QDialog(self)
        dialog.setWindowTitle('Shortcut')
        dialog.resize(300, 300)  # 팝업 크기 조정
        
        layout = QHBoxLayout()
        
        register_animal_button = QPushButton('Register Animal')
        register_food_button = QPushButton('Register Food')

        # 버튼 크기 조정
        register_animal_button.setFixedSize(120, 80)
        register_food_button.setFixedSize(120, 80)      
          
        register_animal_button.clicked.connect(lambda: self.goto_page(Pages.PAGE_REGISTER_ANIMAL.value, dialog))
        register_food_button.clicked.connect(lambda: self.goto_page(Pages.PAGE_REGISTER_FOOD.value, dialog))
        layout.addWidget(register_animal_button)
        layout.addWidget(register_food_button)
        
        dialog.setLayout(layout)
        dialog.exec_()
    
    def goto_page(self, page_index, dialog):
        self.stackedWidget.setCurrentIndex(page_index)
        dialog.accept()
                
    def setFacilitySchedule(self):
        # 현재까지 체크된 체크박스와 해당 체크박스가 위치한 행에 있는 시간들을 모아둘 리스트 초기화
        checked_times = []

        # 체크된 체크박스를 확인하고, 해당 행에 있는 시간을 checked_times에 추가
        for row in range(self.ventilation_table.rowCount()):
            item = self.ventilation_table.item(row, 1)  # 해당 셀의 아이템 가져오기
            if item is not None and item.checkState() == Qt.Checked:  # 아이템이 있고 체크되어 있는 경우
                time_item = self.ventilation_table.item(row, 0)  # 해당 행의 시간 아이템 가져오기
                if time_item is not None:  # 시간 아이템이 있는 경우
                    checked_times.append(time_item.text())  # 시간을 checked_times 리스트에 추가

        # 중복된 값을 제거하여 데이터베이스에 저장
        checked_times = list(set(checked_times))
        self.data_manage.clearScheduledTimes('facility_scheduled_time')
        self.data_manage.addScheduledTimes('facility_scheduled_time', checked_times)
    
    
    def display_existing_schedule(self):
        self.clear_facility_table()
        reserved_times = self.data_manage.get_reserved_times_facility()
        for time in reserved_times:
            time_str = str(time[0])  # 시간 데이터를 문자열로 변환
            hour_minute = time_str.split(":")[:2]  # 시와 분 부분 추출
            hour_minute_str = ":".join([f"{int(part):02d}" for part in hour_minute])  # 시와 분을 두 자리로 표현하여 문자열로 조합

            # 해당 시간을 포함하는 셀을 찾습니다.
            items = self.ventilation_table.findItems(hour_minute_str, Qt.MatchExactly)
            if items:
                item = items[0]
                row = item.row()
                checkbox_item = self.ventilation_table.item(row, 1)  # 같은 행에 있는 체크박스 가져오기
                checkbox_item.setCheckState(Qt.Checked)
            else:
                print("Checkbox for time", hour_minute_str, "not found.")
     
    def clear_facility_table(self):
        # 테이블 위젯의 모든 체크박스를 초기화합니다.
        for row in range(self.ventilation_table.rowCount()):
            checkbox_item = self.ventilation_table.item(row, 1)
            checkbox_item.setCheckState(Qt.Unchecked)    

    def populate_table(self):
        selected_room = self.set_room_box.currentText()
        self.clear_food_table()

        reserved_times = self.data_manage.get_reserved_times(selected_room)
        for time in reserved_times:
            time_str = str(time[0])  # 시간 데이터를 문자열로 변환
            hour_minute = time_str.split(":")[:2]  # 시와 분 부분 추출
            hour_minute_str = ":".join([f"{int(part):02d}" for part in hour_minute])  # 시와 분을 두 자리로 표현하여 문자열로 조합

            # 해당 시간을 포함하는 셀을 찾습니다.
            items = self.feeding_table.findItems(hour_minute_str, Qt.MatchExactly)
            if items:
                item = items[0]
                row = item.row()
                checkbox_item = self.feeding_table.item(row, 1)  # 같은 행에 있는 체크박스 가져오기
                checkbox_item.setCheckState(Qt.Checked)
            else:
                print("Checkbox for time", hour_minute_str, "not found.")

    def clear_food_table(self):
        # 테이블 위젯의 모든 체크박스를 초기화합니다.
        for row in range(self.feeding_table.rowCount()):
            checkbox_item = self.feeding_table.item(row, 1)
            checkbox_item.setCheckState(Qt.Unchecked)

    def setTableWidget(self,tableWidget):
        tableWidget.setRowCount(24)
        tableWidget.setColumnCount(2)
        tableWidget.horizontalHeader().setStretchLastSection(True)

        for hour in range(24):
            time_item = QTableWidgetItem(f"{hour:02d}:00")
            time_item.setFlags(time_item.flags() ^ Qt.ItemIsEditable)
            tableWidget.setItem(hour, 0, time_item)

        for row in range(24):
            checkbox_item = QTableWidgetItem()
            checkbox_item.setFlags(checkbox_item.flags() | Qt.ItemIsUserCheckable)
            checkbox_item.setCheckState(Qt.Unchecked)
            tableWidget.setItem(row, 1, checkbox_item)

        tableWidget.setHorizontalHeaderLabels(["Time", "Assigned"])

    def on_set_food_button_click(self):
        selected_room = self.set_room_box.currentText()
        self.data_manage.clearFoodScheduledTimes(selected_room)
        for row in range(self.feeding_table.rowCount()):
            checkbox_item = self.feeding_table.item(row, 1)
            if checkbox_item.checkState() == Qt.Checked:
                room_item = self.feeding_table.item(row, 0)
                room_number = room_item.text()
                self.data_manage.insert_food_schedule(selected_room, room_number)
        
    
    def update_facility_setting(self):
        # 업데이트할 새 설정 (예시로 값 지정)
        new_settings = {
            'water_tank_warning_level': self.water_tank_warning_level_box.currentText(),
            'water_tank_charging_level': self.water_tank_charging_level_box.currentText(),
            'water_container_warning_level':self.water_container_warning_level_box.currentText(),
            'food_tank_warning_level': self.food_tank_warning_level_box.currentText(),
            'food_tank_charging_level': self.food_tank_charging_level_box.currentText() 
        }
        self.data_manage.updateFacilitySetting(new_settings)        
        self.facility_setting_df=self.data_manage.getFacilitySetting()
        self.load_data_to_table(self.facility_table,self.facility_setting_df)
            # 버튼 클릭 시, 테이블명을 인수로 전달
        
        
          
        
        
    def register_new_animal(self):
        rfid_uid=self.RFID_edit.text()
        animal_id=self.ID_edit.text()
        gender =self.select_gender_box.currentText()
        age=self.age_edit.text()
        food_brand=self.select_feed_box.currentText()
        room =self.select_room_box.currentText()
        weight=self.weight_edit.text()
        registered_date=self.registered_date_animal.text()
        self.data_manage.register_animal(animal_id, gender, age, food_brand, room , weight, registered_date, rfid_uid)
        self.annimal_df=self.data_manage.getAnimal()
        self.label_33.setText("새로운 동물 등록이 완료되었습니다.")

                
    def register_new_food(self):
        barcode_id = self.barcode_edit.text()
        brand_name = self.brand_name_edit.text()
        weight = self.feed_weight_edit.text()
        expiry_date = self.expiry_date_edit.text()
        registered_date = self.registered_date_food.text() 
        self.data_manage.register_food(barcode_id, brand_name, weight, expiry_date, registered_date)
        self.food_df=self.data_manage.getFood()
        self.label_35.setText("새로운 사료 등록이 완료되었습니다.")
        self.food_df.rename(columns={'weight': 'weight (kg)'}, inplace=True)
        self.load_data_to_table(self.search_food_table, self.food_df)
        # food_df 컬럼 데이터를 ComboBox에 추가
        self.load_combobox(self.food_df, 'barcode_id', self.search_food_id_box)
        self.load_combobox(self.food_df, 'brand_name', self.search_brand_name_box)
        self.load_combobox(self.food_df, 'weight (kg)', self.search_food_weight_box)
        self.load_combobox(self.food_df, 'registered_date', self.search_registered_date_box)
        self.load_combobox(self.food_df, 'expiry_date', self.search_expiry_date_box)
        
                        
    def register_new_harmful_animal(self):
        animal_name= self.harmful_animal_name_edit.text()
        index_num = self.harmful_animal_index_box.currentText()
        self.data_manage.register_harmful_animal(index_num,animal_name)
        self.harmful_animal_df=self.data_manage.getHarmfulAnimal()
        self.load_data_to_table(self.harmful_animal_table, self.harmful_animal_df)
        self.refresh_combo_box()
    
                
    def refresh_combo_box(self):
        # harmful_animal_df 데이터프레임에서 index_num 열의 모든 값 추출
        used_numbers = self.harmful_animal_df['index_num'].tolist()
        all_numbers = list(range(1, 11))
        unused_numbers = [num for num in all_numbers if num not in used_numbers]
        self.harmful_animal_index_box.clear()
        for num in unused_numbers:
            self.harmful_animal_index_box.addItem(str(num))    
                
    def register_new_employee(self):
        employee_name = self.employee_name_edit.text()
        registered_date = self.registered_date_employee.text()        
        self.data_manage.register_employee(employee_name, registered_date)
        self.employee_df= self.data_manage.getEmployeeData()
        self.load_data_to_table(self.registered_employee_table, self.employee_df)
        
    def update_captured_date_end(self):
        selected_start_date = self.captured_date_start.currentText()
        self.captured_date_end.clear()
        
        if selected_start_date != 'all':
            self.captured_date_end.addItem('all')
            selected_start_date = datetime.strptime(selected_start_date, '%Y-%m-%d').date()
            filtered_dates = self.camera_df[self.camera_df['captured_date'] >= selected_start_date]['captured_date'].unique()
            sorted_filtered_dates = sorted(filtered_dates)
            self.captured_date_end.addItems(map(str, sorted_filtered_dates))
        else:
            self.captured_date_end.addItem('all')
    
    def animal_search(self):
        filters = { 
         'animal_id': self.search_id_box.currentText(),
         'gender': self.search_gender_box.currentText(),
         'age (in months)': self.search_age_box.currentText(),
         'food_brand': self.search_food_brand_box.currentText(),
         'room': self.search_room_box.currentText(),
         'weight (kg)': self.search_weight_box.currentText(),
         'registered_date': self.search_rfid_box.currentText(),
         'rfid_uid': self.search_animal_date_box.currentText()
        }
        filtered_df = self.animal_df.copy()
        for key, value in filters.items():
            if value != 'all':
                filtered_df = filtered_df[filtered_df[key].astype(str) == value]
        self.load_data_to_table(self.search_animal_table, filtered_df)           

    def food_search(self):
        filters = {
            'barcode_id': self.search_food_id_box.currentText(),
            'brand_name': self.search_brand_name_box.currentText(),
            'weight (kg)': self.search_food_weight_box.currentText(),
            'registered_date': self.search_registered_date_box.currentText(),
            'expiry_date': self.search_expiry_date_box.currentText()
        }
        filtered_df = self.food_df.copy()
        for key, value in filters.items():
            if value != 'all':
                filtered_df = filtered_df[filtered_df[key].astype(str) == value]
        self.load_data_to_table(self.search_food_table, filtered_df)
    
    def camera_search(self):
        filters = { 
            'cam_num': self.select_camera_box_video.currentText(),
            'info_type': self.select_camera_type_box.currentText(),
        }
        start_date_str = self.captured_date_start.currentText()
        end_date_str = self.captured_date_end.currentText()
        filtered_df = self.camera_df.copy()
        for key, value in filters.items():
            if value != 'all':
                filtered_df = filtered_df[filtered_df[key].astype(str) == value]
        if start_date_str != 'all':
            start_date = datetime.strptime(start_date_str, '%Y-%m-%d').date()
            filtered_df = filtered_df[filtered_df['captured_date'] >= start_date]
        if end_date_str != 'all':
            end_date = datetime.strptime(end_date_str, '%Y-%m-%d').date()
            filtered_df = filtered_df[filtered_df['captured_date'] <= end_date]

        # 결과를 테이블에 로드
        self.load_data_to_table(self.search_camera_table, filtered_df)
 
    
   
        
    def load_combobox(self, df, column_name, combobox):
        combobox.clear()
        unique_values = np.sort(df[column_name].unique())
        combobox.addItem('all')
        combobox.addItems(map(str, unique_values))

    def load_combobox_except_all(self, df, column_name, combobox):
        combobox.clear()
        unique_values = np.sort(df[column_name].unique())
        combobox.addItems(map(str, unique_values))    
        
        

    def auto_resize_columns(self, table):
        table.resizeColumnsToContents()
        header = table.horizontalHeader()
        for col in range(table.columnCount()):
            header_text_width = table.fontMetrics().horizontalAdvance(table.horizontalHeaderItem(col).text())
            table.setColumnWidth(col, max(table.columnWidth(col), header_text_width + 20))  # 약간의 여유 공간 추가
        table.horizontalHeader().setStretchLastSection(True)
        header.setFixedHeight(25)  
                                     
    def load_data_to_table(self, table, df):
        # 행과 열 설정
        table.setRowCount(df.shape[0])
        table.setColumnCount(df.shape[1])
        # 컬럼 헤더 설정
        table.setHorizontalHeaderLabels(df.columns)
        # 데이터 삽입
        for i in range(df.shape[0]):
            for j in range(df.shape[1]):
                table.setItem(i, j, QTableWidgetItem(str(df.iat[i, j])))
        
           

    def convert_to_torchtensor(self, num_data):
        transform = transforms.ToTensor()
        tensor_data = transform(num_data)
        return tensor_data  
    
    def move_to_device(self, data):
        # GPU로 이동
        tensor_data = data.to(self.device)
        return tensor_data
    
    
    def updateDetectedListWithYolo(self, frame_for_yolo):

        self.yolo_detect_class.clear()
        self.yolo_detect_class_coordinate.clear()

        frame_for_yolo = self.convert_to_torchtensor(frame_for_yolo)
        frame_for_yolo = self.move_to_device(frame_for_yolo)

        results = self.model.predict(source=frame_for_yolo, classes=19, conf = 0.2)
        names = self.model.names
        
        for r in results:
            for idx,cls_name in enumerate(r.boxes.cls):
                tmp_name = names[int(cls_name.item())]
                self.yolo_detect_class.append(tmp_name)
                self.yolo_detect_class_coordinate.append(r.boxes.xyxy.cpu())

        print(self.yolo_detect_class)

    
    def updateHarmfulAnimalDetectedListWithYolo(self, frame_for_yolo):

        self.yolo_harmful_animal_detect_class.clear()
        self.yolo_harmful_animal_detect_class_coordinate.clear()

        frame_for_yolo = self.convert_to_torchtensor(frame_for_yolo)
        frame_for_yolo = self.move_to_device(frame_for_yolo)

        results = self.model.predict(source=frame_for_yolo, classes=[16, 21], conf = 0.2)
        names = self.model.names
        
        for r in results:
            for idx,cls_name in enumerate(r.boxes.cls):
                tmp_name = names[int(cls_name.item())]
                self.yolo_harmful_animal_detect_class.append(tmp_name)
                self.yolo_harmful_animal_detect_class_coordinate.append(r.boxes.xyxy.cpu())

        print(self.yolo_harmful_animal_detect_class)


    def drawRedBox(self,img_form):
        result = img_form.copy()

        for val in self.yolo_detect_class_coordinate:
            for idx, coord in enumerate(val):
                x1 = coord[0]
                y1 = coord[1]
                x2 = coord[2]
                y2 = coord[3]

                roi = result[int(y1):int(y2), int(x1):int(x2)]
                # result[int(y1):int(y2), int(x1):int(x2)] = self.checkAnimalTemepature(roi)
                self.checkAnimalTemperature(roi)

                #소만 crop한 이미지
                # cv2.imshow('ROI', roi)

                cv2.rectangle(result, (int(x1.item()), int(y1.item())), (int(x2.item()), int(y2.item())), (255, 0, 0), 2)
                
                font = cv2.FONT_HERSHEY_SIMPLEX
                font_scale = 0.5
                font_thickness = 1
                font_color = (255, 255, 255)  # 흰색
                
                text_size, _ = cv2.getTextSize(self.yolo_detect_class[idx], font, font_scale, font_thickness)
                text_x = int((x1 + x2) / 2 - text_size[0] / 2)
                text_y = int(y2 + text_size[1] + 5)  # 객체 아래에 위치
                cv2.putText(result, self.yolo_detect_class[idx], (text_x, text_y), font, font_scale, font_color, font_thickness)
                
        return result 
    
    def drawBlueBox(self,img_form,detect_class, coordinate):
        result = img_form.copy()

        for val in coordinate:
            for idx, coord in enumerate(val):
                x1 = coord[0]
                y1 = coord[1]
                x2 = coord[2]
                y2 = coord[3]

                roi = result[int(y1):int(y2), int(x1):int(x2)]
                # result[int(y1):int(y2), int(x1):int(x2)] = self.checkAnimalTemepature(roi)
                self.checkAnimalTemperature(roi)

                #소만 crop한 이미지
                # cv2.imshow('ROI', roi)

                cv2.rectangle(result, (int(x1.item()), int(y1.item())), (int(x2.item()), int(y2.item())), (0, 0, 255), 2)
                
                font = cv2.FONT_HERSHEY_SIMPLEX
                font_scale = 0.5
                font_thickness = 1
                font_color = (255, 255, 255)  # 흰색
                
                text_size, _ = cv2.getTextSize(detect_class[idx], font, font_scale, font_thickness)
                text_x = int((x1 + x2) / 2 - text_size[0] / 2)
                text_y = int(y2 + text_size[1] + 5)  # 객체 아래에 위치
                cv2.putText(result, detect_class[idx], (text_x, text_y), font, font_scale, font_color, font_thickness)
                
        return result 
    
        
    def checkRemainedFood(self,img_form):
        result = img_form.copy()
        
        #HSV 색상으로 변환
        hsv_image = cv2.cvtColor(result, cv2.COLOR_BGR2HSV)

        # 노란색 범위 정의
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])
        
        # 초록색 범위 정의
        lower_green = np.array([35, 100, 100])
        upper_green = np.array([85, 255, 255])

        # 각각의 색상 범위 내의 픽셀 분리
        mask_yellow = cv2.inRange(hsv_image, lower_yellow, upper_yellow)
        mask_green = cv2.inRange(hsv_image, lower_green, upper_green)

        # 두 마스크 결합
        combined_mask = cv2.bitwise_or(mask_yellow, mask_green)

        # # 마스크 생성
        # mask = cv2.inRange(hsv_image, lower_limit, upper_limit)

        # 마스크를 이미지로 변환
        masked_image = cv2.bitwise_and(result, result, mask=combined_mask)


        #마스크 이미지 띄우기
        # cv2.imshow('masked', masked_image)

        # 각 이미지가 차지하는 픽셀 수 계산
        cropped_pixel_count = np.count_nonzero(result)
        masked_pixel_count = np.count_nonzero(masked_image)

        remained_feed = (masked_pixel_count / cropped_pixel_count) * 100
        
        result_str = "remain food rate : {:.1f}".format(remained_feed)

        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.5
        font_thickness = 1
        font_color = (255, 255, 255)  # 흰색
                         
        text_size, _ = cv2.getTextSize(result_str, font, font_scale, font_thickness)
        #label의 width와 height 가져오기 
        w = self.monitor_camera_label.width()
        h = self.monitor_camera_label.height()
        text_x = int(w /2) -100
        text_y = int(h /2) -170
        cv2.putText(result, result_str, (text_x, text_y), font, font_scale, font_color, font_thickness)

        #print(result_str)

        # 마스크 확인해 보고 싶다면        
        # result = masked_image

        return result 
    
    def checkAnimalTemperature(self,cropped_img):
        result = cropped_img.copy()
        
        #HSV 색상으로 변환
        hsv_image = cv2.cvtColor(result, cv2.COLOR_RGB2HSV)
        bgr_image = cv2.cvtColor(result, cv2.COLOR_RGB2BGR)
        # 빨간색과 분홍색의 HSV 범위 정의
        lower_red_pink1 = np.array([0, 70, 50])
        upper_red_pink1 = np.array([10, 255, 255])
        lower_red_pink2 = np.array([160, 70, 50])
        upper_red_pink2 = np.array([180, 255, 255])
        
        # 두 범위의 마스크 생성
        mask_red1 = cv2.inRange(hsv_image, lower_red_pink1, upper_red_pink1)
        mask_red2 = cv2.inRange(hsv_image, lower_red_pink2, upper_red_pink2)
        
        # 두 마스크를 합쳐서 최종 마스크 생성
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)
        
        # 원본 이미지와 마스크를 사용하여 빨간색과 분홍색 부분 추출
        red_pink_only = cv2.bitwise_and(bgr_image, bgr_image, mask=mask_red)
        
        # 파란색과 하늘색의 HSV 범위 정의
        lower_blue_cyan1 = np.array([90, 50, 50])
        upper_blue_cyan1 = np.array([110, 255, 255])
        lower_blue_cyan2 = np.array([80, 50, 70])
        upper_blue_cyan2 = np.array([100, 255, 255])
        
        # 두 범위의 마스크 생성
        mask_blue1 = cv2.inRange(result, lower_blue_cyan1, upper_blue_cyan1)
        mask_blue2 = cv2.inRange(result, lower_blue_cyan2, upper_blue_cyan2)
        
        # 두 마스크를 합쳐서 최종 마스크 생성
        mask_blue = cv2.bitwise_or(mask_blue1, mask_blue2)
        
        # 원본 이미지와 마스크를 사용하여 파란색과 하늘색 부분 추출
        blue_cyan_only = cv2.bitwise_and(bgr_image, bgr_image, mask=mask_blue)

        cv2.imshow('red', red_pink_only)
        cv2.imshow('blue', blue_cyan_only)

        # 각 이미지가 차지하는 픽셀 수 계산
        cropped_pixel_count = np.count_nonzero(result)
        red_masked_pixel_count = np.count_nonzero(red_pink_only)
        blue_masked_pixel_count = np.count_nonzero(blue_cyan_only)

        # print("pixel count")
        # print(cropped_pixel_count)
        # print(masked_pixel_count)
        
        red_area = (red_masked_pixel_count / cropped_pixel_count) * 100
        blue_area = (blue_masked_pixel_count / cropped_pixel_count) * 100
        

        result_str = "red area : {:.1f}, blue area : {:.1f}".format(red_area,blue_area)

        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.1
        font_thickness = 1
        font_color = (255, 255, 255)  # 흰색
                         
        #label의 width와 height 가져오기 
        w = self.monitor_camera_label.width()
        h = self.monitor_camera_label.height()
        text_x = int(h /2) 
        text_y = int(w /2)
        cv2.putText(result, result_str, (text_x, text_y), font, font_scale, font_color, font_thickness)

        print(result_str)

        # 마스크 확인해 보고 싶다면        
        # result = masked_image

        return result 
    def capture(self):
        if not os.path.exists('captures'):
            os.makedirs('captures')
        now = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = 'captures/' + now + '.png'
        # BGR to RGB conversion
        rgb_image = cv2.cvtColor(self.image, cv2.COLOR_BGR2RGB)

        cv2.imwrite(filename, rgb_image)
        
    def updateRecording(self):
        rgb_image = cv2.cvtColor(self.image, cv2.COLOR_BGR2RGB)
        self.writer.write(rgb_image)
        

    def clickRecord(self):
        if not self.isRecStart:
            self.btnRecord.setText("Rec Stop")
            self.isRecStart = True
            self.recordingStart()
        else:
            self.btnRecord.setText("Rec Start")
            self.isRecStart = False
            self.recordingStop()

    def recordingStart(self):
        if not os.path.exists('captures'):
            os.makedirs('captures')ㄴ
        self.record.running = True
        self.record.start()
        now = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = 'captures/' + now + '.avi'
        self.fourcc = cv2.VideoWriter_fourcc(*'XVID')

        w = int(self.temp_cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(self.temp_cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.writer = cv2.VideoWriter(filename, self.fourcc, 20.0, (w, h))
        self.record_label.show()

    def recordingStop(self):
        self.record.running = False
        self.writer.release() 
        self.record_label.hide()
        

    def combochanged(self):
        self.cam_num = self.select_camera_box.currentIndex()

    def updateCameraView(self):
        
        if self.stackedWidget.currentIndex() != 4:
            return

        retval_list = []
        image_list = []

        for idx,cam in enumerate(self.camera_list):
            if cam.isOpened():
                retval, image = cam.read()
                
                retval_list.append(retval)
                image_list.append(image)


        if self.cam_num < len(image_list):

            if retval_list[self.cam_num]:
                self.image = cv2.cvtColor(image_list[self.cam_num],cv2.COLOR_BGR2RGB)

                # if self.cam_num != 0:    # Entrance 카메라를 제외하고 나머지는 cow yolo 인식 처리 하기 
                self.updateDetectedListWithYolo(self.image)
                self.image = self.drawRedBox(self.image)
                self.image = self.checkRemainedFood(self.image)

                # self.updateHarmfulAnimalDetectedListWithYolo(self.image)
                # self.image = self.drawBlueBox(self.image,self.yolo_harmful_animal_detect_class, self.yolo_harmful_animal_detect_class_coordinate)
                
                #이미지 화면에 띄우기
                h,w,c = self.image.shape
                qimage = QImage(self.image.data, w, h, w*c, QImage.Format_RGB888)

                self.pixmap = self.pixmap.fromImage(qimage)
                self.pixmap = self.pixmap.scaled(self.monitor_camera_label.width(), self.monitor_camera_label.height())

                self.monitor_camera_label.setPixmap(self.pixmap)
            else:
                print(f"Camera cannot be opened")

    def cameraStart(self):
        self.camera.running = True
        self.camera.start()
        
        

    def cameraStop(self):
        self.camera.running = False
        for cam in self.camera_list:
            cam.release


    def decryption(self,data,key,tag):
        # 복호화
        cipher = AES.new(key, AES.MODE_EAX, nonce=cipher.nonce)
        decrypted_data = cipher.decrypt_and_verify(data, tag)

        print("Decrypted text:", decrypted_data.decode('utf-8'))

    
    def checkScheduleForTaskAssig(self):
        pass
    
    def assignRobotTask(self):

        #로봇이 이미 모두 할당 되었으면 배정할 필요 x
        assign_robot = 0
        for robot in self.robot_list:
            if robot.task_id != 0 : #할당 되어 있다 
                assign_robot += 1
            
        
        #모든 로봇이 할당 되어 있어서 return
        if assign_robot == len(self.robot_list) :
            # print("All robot is assigned")
            return

        #할당할 로봇이 있다면     

        current_datetime = datetime.now()
        busy_task_time = current_datetime
        busy_index = 0

        if len(self.task_list) > 0:

            #가장 빨리 할당해야 하는 task 찾기 
            
            for idx, task in enumerate(self.task_list):
                if task.task_time <= busy_task_time:
                    busy_task_time = task.task_time
                    busy_index = idx
            
            # task type 맞는 지 확인하고 task 할당
            for robot in self.robot_list:
                if robot.task_id == 0 : #할당이 안되어 있다
                    if robot.robot_type == self.task_list[busy_index].task_type:
                        robot.setTaskID(self.task_list[busy_index].task_id)
                        print("robot_num", end="")
                        print(robot.robot_num)
                        print("status: assigned")
                        self.task_list[busy_index].updateTaskProgress() 
                        # print("robot_num : ", end="")
                        # print(robot.robot_num)
                        # print("assigend")
                        return

        else:
            # print("empty")
            return
        
        # if current_datetime < modified_datetime:
        #     print("현재 시간은 수정된 시간보다 이전입니다.")
        # elif current_datetime > modified_datetime:
        #     print("현재 시간은 수정된 시간보다 이후입니다.")
        # else:
        #     print("현재 시간은 수정된 시간과 같습니다.")

    def isServiceCalled(self, robot_task_id):
        for task in self.task_list:
            if task.task_id == robot_task_id:
                if task.getCurrentProgress() == 1:
                    print("service call")
                    return True
        return False
    
    def isNavArucoFinished(self):
        # if task.getCurrentProgress() == 1:
        #     print("nav")
            return True
    
    def isArucoFoodTank(self):
        return True

    def isArucoStation(self):
        return True
    
    def isFoodChargeDone(self):
        return True
    
    def isArucoBarnEntrance(self):
        return True
    
    def isArucoDistanceSatisfied(self):
        return True
    
    def isFoodDistributeDone(self):
        return True
    
    def isTaskDone(self, robot_task_id):
        for task in self.task_list:
            if task.task_id == robot_task_id:
                if task.getCurrentProgress() == 6:
                    return True
        return False

    def robotStatusManager(self):

        for robot in self.robot_list:
            if robot.status == Status.STATUS_STANDBY.value:
                if self.isServiceCalled(robot.getTaskID()) == True:
                    robot.setStatus(Status.STATUS_NAV_ARUCO.value)

                elif self.isArucoFoodTank() == True:
                    robot.setStatus(Status.STATUS_FOOD_CHARGE.value)

                elif self.isArucoBarnEntrance() == True:
                    robot.setStatus(Status.STATUS_MANUAL_MOVE.value)

                elif self.isArucoDistanceSatisfied() == True:
                    robot.setStatus(Status.STATUS_FOOD_DISTRIBUTE.value)
                
                elif self.isTaskDone() == True:
                    robot.setStatus(Status.STATUS_RETURN.value)
                

            elif robot.status == Status.STATUS_NAV_ARUCO.value:
                if self.isNavArucoFinished() == True:
                    robot.setStatus(Status.STATUS_STANDBY.value)

            elif robot.status == Status.STATUS_FOOD_CHARGE.value:
                if self.isFoodChargeDone() == True:
                    robot.setStatus(Status.STATUS_STANDBY.value)
                    
            elif robot.status == Status.STATUS_MANUAL_MOVE.value:
                if self.isArucoDistanceSatisfied() == True:
                    robot.setStatus(Status.STATUS_STANDBY.value)

            elif robot.status == Status.STATUS_FOOD_DISTRIBUTE.value:
                if self.isFoodDistributeDone() == True:
                    robot.setStatus(Status.STATUS_STANDBY.value)

            elif robot.status == Status.STATUS_RETURN.value:
                if self.isArucoStation() == True:
                    robot.setStatus(Status.STATUS_STANDBY.value)
                    

            
    
    def checkTaskProgress(self):
        for task in self.task_list:
            if task.getCurrentProgress() == 0:
                return
            elif task.getCurrentProgress() == 1:
                self.sendToFoodTank(task.getFoodTank())
            elif task.getCurrentProgress() == 2:
                self.sendSignaltoFoodTank(task.getFoodTank())
            elif task.getCurrentProgress() == 3:
                self.sendToBarnEntrance()
            elif task.getCurrentProgress() == 4:
                pass
            elif task.getCurrentProgress() == 5:
                pass


    def sendToBarnEntrance(self):
        #service call 해서 축사 앞으로 보내기 
        pass

    def sendSignaltoFoodTank(self, tank_num):
        #사료탱크에 신호줘서 사료 받기
        pass

    def sendToFoodTank(self, tank_num):
        #service call 해서 tank_num에 맞게 ㄱㄱ
        pass
                
    def update_robot_thread(self):
        self.assignRobotTask()
        self.robotStatusManager()
        self.checkTaskProgress()

        
    def get_food_type(self):
        food_tank_type = 0
        return food_tank_type
    
    def task_add_button_clicked(self):
        #task 클래스로 객체 만들어서 list에 넣어주기
        current_datetime = datetime.now()
        modified_datetime = current_datetime.replace(hour=int(self.hour_edit.text()), minute=int(self.minutes_edit.text()), second=0, microsecond=0)
        self.task_id += 1
        temp_task = Task(self.task_id,TaskScheduleType.TASK_REGISTERED.value, self.room_number_edit.text(),modified_datetime)
        temp_task.setFoodTank(self.get_food_type())
        self.task_list.append(temp_task)

    def foodrobot_up_button_clicked(self):
        teleop_node.twist.linear.x = 3.0
        teleop_node.twist.angular.z = 0.0
        teleop_node.publish_twist()
    
    def foodrobot_down_button_clicked(self):
        teleop_node.twist.linear.x = -3.0
        teleop_node.twist.angular.z = 0.0
        teleop_node.publish_twist()

    def foodrobot_left_button_clicked(self):
        teleop_node.twist.linear.x = 0.0
        teleop_node.twist.angular.z = 3.0
        teleop_node.publish_twist()

    def foodrobot_right_button_clicked(self):
        teleop_node.twist.linear.x = 0.0
        teleop_node.twist.angular.z = -3.0
        teleop_node.publish_twist()    

    def foodtrailer_servo_open_button_clicked(self):
        self.send_to_rasp("FT1,1")
        print("FT1,1 send")

    def foodtrailer_servo_close_button_clicked(self):
        self.send_to_rasp("FT1,0")
        print("FT1,0 send")

    def foodtank_servo_open_button_clicked(self):
        self.send("FT1,1")
        print("FT1,1 send")

    def foodtank_servo_close_button_clicked(self):
        self.send("FT1,0")
        print("FT1,0 send")

    def send(self, data=""):
        #send
        # response = str(data)
        print(data)
        ServerThread.client_socket_list.send(data.encode("utf-8"))

    def send_to_rasp(self, data=""):
        #send
        # response = str(data)
        print(data)
        ServerThread.client_socket_list.send(data.encode("utf-8"))

    def closeEvent(self, event):
        #창을 종료할때 task_id 저장해주기
        self.yaml_file.saveYamlFile(self.task_id)

        self.robotThreadStop()

        if self.server_thread:
            self.server_thread.stop()
        self.cameraStop()
        event.accept()
        

    def robotThreadStart(self):
        self.robot_thread.start()

    def robotThreadStop(self):
        self.robot_thread.stop()

    def start_tcp_server_thread(self):
        host = self.ip_input.text()
        port = int(self.port_input.text())
        self.server_thread = ServerThread(host, port)
        self.server_thread.start()   
        self.server_thread.image_signal.connect(self.update_image)
        self.server_thread.new_connection.connect(self.update_client_info)
        self.server_thread.connection_lost.connect(self.remove_client_info)
        self.server_thread.start()
        self.server_label.show()
        self.connect_button.hide()
        self.quit_button.show()
        
    def update_image(self, q_img):
        pixmap = QPixmap.fromImage(q_img)
        self.robot_label.setPixmap(pixmap)
        
    def stop_tcp_server_thread(self):
        if self.server_thread:
            self.server_thread.stop()  # 서버 스레드 종료
            self.server_thread = None  # 서버 스레드 객체를 None으로 설정하여 참조 제거
            self.server_label.hide()  # 서버 레이블 숨김
            self.connect_button.show()  # connect_button 다시 표시
            self.quit_button.hide()  # quit_button 숨김
            self.ip_input.clear()
            self.port_input.clear()
        
    @QtCore.pyqtSlot(str, int)
    def update_client_info(self, ip, port):
        self.client_label.show()
        new_row = pd.DataFrame([{'IP': ip, 'Port': port}])
        self.client_df = pd.concat([self.client_df, new_row], ignore_index=True)
        self.display_client_list()

    
    @QtCore.pyqtSlot(str, int)
    def remove_client_info(self, ip, port):
        print("클라이언트 연결이 끊어졌습니다.")
        self.client_df = self.client_df[(self.client_df['IP'] != ip) | (self.client_df['Port'] != port)]
        self.display_client_list()    
        
    def display_client_list(self):
        self.client_table.setColumnCount(2)
        self.client_table.setRowCount(len(self.client_df))
        # DataFrame을 순회하며 각 클라이언트의 IP와 Port 정보를 테이블에 추가
        for row, (index, row_data) in enumerate(self.client_df.iterrows()):
            self.client_table.setItem(row, 0, QTableWidgetItem(row_data['IP']))  # 첫 번째 열에 IP 설정
            self.client_table.setItem(row, 1, QTableWidgetItem(str(row_data['Port'])))  # 두 번째 열에 Port 설정   
        # DataFrame이 비어 있으면 클라이언트 라벨을 숨기고, 그렇지 않으면 보여줌
        if self.client_df.empty:
            self.client_label.hide()
        else:
            self.client_label.show()

    
    def nav_to_station1_button_clicked(self):
        
        service_name = '/nav_service'
        cli = self.service_client_node.create_client(NavToPose, service_name)
        req = NavToPose.Request()
        req.x = 0.007822726853191853
        req.y = -0.024536626413464546
        req.z = 0.002471923828125
        
        print(req)

        while not cli.wait_for_service(timeout_sec=1.0):
            print("Waiting for service")

        future = cli.call_async(req)

        while not future.done():
            rp.spin_once(self.service_client_node)
            print(future.done(), future.result())


    def logout_button_clicked(self):
        self.stackedWidget.setCurrentIndex(Pages.PAGE_LOGIN.value)
        self.input_id =""; self.input_pw =""
        self.id_input.setText(""); self.pw_input.setText("")
        self.UserEdit.hide()
        self.is_logged_in = False
        self.update_buttons() 
        
    
    def login_button_clicked(self):
        self.input_id = int(self.id_input.text())
        self.input_pw = int(self.pw_input.text())
        user_row = self.userdata_df[self.userdata_df['id'] == self.input_id]
        if not user_row.empty and user_row.iloc[0]['password'] == self.input_pw:
            self.login_success(user_row)
        else:
            self.login_failed()

    def login_success(self, user_row):
        permission = user_row.iloc[0]['permission']
        username = user_row.iloc[0]['username']
        self.stackedWidget.setCurrentIndex(Pages.PAGE_HOME.value)
        self.login_information_label.setText(f"[Permission: {permission}] {username} is logged in")
        self.UserEdit.show()
        self.UserEdit.setText(f"User: {username} ({permission})")
        self.is_logged_in = True
        self.update_buttons()  # 버튼 업데이트
        
        # Admin 권한이면 설정 버튼 활성화
        if permission == 'Admin':
            self.setting_button.setEnabled(True)
        else:
            self.setting_button.setEnabled(False)

    def login_failed(self):
        self.is_logged_in = False
        self.login_result_label.setText('login failed')
        self.update_buttons()  # 버튼 업데이트
            
    def update_buttons(self):
        for button in self.buttons:
            button.setEnabled(self.is_logged_in)

            
    # def toolbox_changed(self):
    #     if self.toolBox.currentIndex() == 5 and self.is_logged_in == True:
    #         self.stackedWidget.setCurrentIndex(Pages.PAGE_LOG.value)
    #     elif self.toolBox.currentIndex() == 6 and self.is_logged_in == True:
    #         self.stackedWidget.setCurrentIndex(Pages.PAGE_SETTING.value)
    
    def log_page_button_clicked(self):
        self.stackedWidget.setCurrentIndex(Pages.PAGE_LOG.value)
            
    def setting_page_button_clicked(self):
        self.stackedWidget.setCurrentIndex(Pages.PAGE_SETTING.value)

    def home_page_button_clicked(self):
        self.stackedWidget.setCurrentIndex(Pages.PAGE_HOME.value)

    def monitor_barnpage_button_clicked(self):
        self.stackedWidget.setCurrentIndex(Pages.PAGE_MONITOR_BARN.value) 
        animal_count = len(self.animal_df)
        self.animalcount_display_label.setText(str(animal_count))
    
    def monitor_facilitiespage_button_clicked(self):
        self.stackedWidget.setCurrentIndex(Pages.PAGE_MONITOR_FACILITIES.value)

    def monitor_camerapage_button_clicked(self):
        self.stackedWidget.setCurrentIndex(Pages.PAGE_MONITOR_CAMERA.value)
    
    def control_robotpage_button_clicked(self):
        self.stackedWidget.setCurrentIndex(Pages.PAGE_CHOOSE_CONTROL_ROBOT.value)

    def control_robotpage(self,robot_name):
        self.stackedWidget.setCurrentIndex(Pages.PAGE_CONTROL_ROBOT.value)
        self.robot_name_label.setText(robot_name)
        
    def control_facilitiespage_button_clicked(self):
        self.stackedWidget.setCurrentIndex(Pages.PAGE_CONTROL_FACILITIES.value)
        
    def robotmanager_choose_button_clicked(self):
        self.stackedWidget.setCurrentIndex(Pages.PAGE_CHOOSE_ROBOTMANAGER_TASK.value)   

    def robotmanager_taskpage_button_clicked(self,robot_name):
        self.stackedWidget.setCurrentIndex(Pages.PAGE_ROBOTMANAGER_TASK.value)
        self.food_robot_label.setText(robot_name)
        now = datetime.now()
        self.hour_edit.setText(str(now.hour))
        self.minutes_edit.setText(str(now.minute))

    def choose_datamanager_animalpage_button_clicked(self):
        self.stackedWidget.setCurrentIndex(Pages.PAGE_CHOOSE_DATAMANAGER_ANIMAL.value)
    
    def register_animal_button_clicked(self):
        self.label_33.setText("등록할 RFID를 리더기 위에 얹어주세요.")
        self.load_combobox_except_all(self.food_df, 'brand_name', self.select_feed_box)
        self.RFID_edit.setText("")
        self.ID_edit.setText("")
        self.age_edit.setText("")
        self.weight_edit.setText("")
        self.stackedWidget.setCurrentIndex(Pages.PAGE_REGISTER_ANIMAL.value)
    
    def search_animal_button_clicked(self):
        self.animal_df=self.data_manage.getAnimal()
        self.animal_df.rename(columns={'age': 'age (in months)', 'weight': 'weight (kg)'}, inplace=True)
        self.load_data_to_table(self.search_animal_table, self.animal_df)
        # food_df 컬럼 데이터를 ComboBox에 추가
        self.load_combobox(self.animal_df, 'animal_id', self.search_id_box)
        self.load_combobox(self.animal_df, 'gender', self.search_gender_box)
        self.load_combobox(self.animal_df, 'age (in months)', self.search_age_box)
        self.load_combobox(self.animal_df, 'food_brand', self.search_food_brand_box)
        self.load_combobox(self.animal_df, 'room', self.search_room_box)
        self.load_combobox(self.animal_df, 'weight (kg)', self.search_weight_box)
        self.load_combobox(self.animal_df, 'registered_date', self.search_rfid_box)
        self.load_combobox(self.animal_df, 'rfid_uid', self.search_animal_date_box)
        self.stackedWidget.setCurrentIndex(Pages.PAGE_SEARCH_ANIMAL.value)
    
    def choose_datamanager_foodpage_button_clicked(self):
        self.stackedWidget.setCurrentIndex(Pages.PAGE_CHOOSE_DATAMANAGER_FOOD.value)

    def register_food_button_clicked(self):
        self.label_35.setText("사료 포대의 바코드를 인식해주세요")
        self.barcode_edit.setText("")
        self.brand_name_edit.setText("")
        self.feed_weight_edit.setText("")
        self.expiry_date_edit.setText("")
        self.stackedWidget.setCurrentIndex(Pages.PAGE_REGISTER_FOOD.value)
    
    def search_food_button_clicked(self):
        self.stackedWidget.setCurrentIndex(Pages.PAGE_SEARCH_FOOD.value)

    def datamanager_videopage_button_clicked(self):
        self.stackedWidget.setCurrentIndex(Pages.PAGE_DATAMANAGER_VIDEO.value)

    def choose_datamanager_facilitiespage_button_clicked(self):
        self.stackedWidget.setCurrentIndex(Pages.PAGE_CHOOSE_DATAMANAGER_FACILITIES.value)

    def register_employee_button_clicked(self):
        self.stackedWidget.setCurrentIndex(Pages.PAGE_REGISTER_EMPLOYEE.value)
    
    def register_others_button_clicked(self):
        self.stackedWidget.setCurrentIndex(Pages.PAGE_REGISTER_OTHERS.value)

    def schedule_foodpage_button_clicked(self):
        self.stackedWidget.setCurrentIndex(Pages.PAGE_SCHEDULE_FOOD.value)

    def schedule_facilitiespage_button_clicked(self):
        self.stackedWidget.setCurrentIndex(Pages.PAGE_SCHEDULE_FACILITIES.value)

   
        
if __name__ == "__main__":
    app = QApplication(sys.argv)
    teleop_node = Teleop()
    ros_thread = RosThread(teleop_node)
    ros_thread.start()
    myWindows = WindowClass()
    myWindows.show()
    sys.exit(app.exec_())