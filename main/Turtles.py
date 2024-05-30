import sys
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5 import uic, QtCore
from PyQt5.QtCore import QThread, pyqtSignal,Qt,QDate
from PyQt5 import QtWidgets, QtCore
import rclpy as rp
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
import numpy as np 

#encryption
from Crypto.Cipher import AES
from Crypto.Random import get_random_bytes



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
    TASK_SCHEDULED = 0
    TASK_REGISTERED = 1

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


class WindowClass(QMainWindow, from_class) :
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.setWindowTitle("Turtles : Herding Heroes")
        rp.init()

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
        
        #databases 연결
        self.data_manage = DBManager("192.168.1.101", "0000", 3306, "turtles", "TurtlesDB")
        self.animal_df = self.data_manage.getAnimal()
        self.camera_df = self.data_manage.getCameraPath()
        self.employee_df= self.data_manage.getEmployeeData()
        self.food_df = self.data_manage.getFood()
        self.schedule_df = self.data_manage.getFoodRobotSchedule()
        self.userdata_df = self.data_manage.getUserData()
        self.harmful_animal_df=self.data_manage.getHarmfulAnimal()
        
        
        
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

        self.toolBox.currentChanged.connect(self.toolbox_changed) ## 버튼 페이지 연결 

        #login logout 버튼 연결
        self.logout_button.clicked.connect(self.logout_button_clicked)
        self.login_button.clicked.connect(self.login_button_clicked)

    
        #robot thread  
        self.robot_thread.update.connect(self.update_robot_thread)

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
        self.setupTableWidget(self.feeding_table); self.setupTableWidget(self.feeding_table_check)
        self.layout.addWidget(self.feeding_table); self.layout.addWidget(self.feeding_table_check)
        self.setupTableWidget(self.ventilation_table); self.setupTableWidget(self.ventilation_table_check)
        self.layout.addWidget(self.ventilation_table); self.layout.addWidget(self.ventilation_table_check)
        self.setLayout(self.layout) 
        
        self.stop_recording_button.hide()
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
        
        self.auto_resize_columns(self.search_animal_table)
        self.auto_resize_columns(self.search_food_table)
        self.auto_resize_columns(self.search_camera_table)
        self.auto_resize_columns(self.registered_employee_table)
        self.auto_resize_columns(self.harmful_animal_table)
        
        
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
        self.load_combobox(self.food_df, 'weight (kg)', self.search_food_weight_box)
        self.load_combobox(self.food_df, 'registered_date', self.search_registered_date_box)
        self.load_combobox(self.food_df, 'expiry_date', self.search_expiry_date_box)
        
        #camera_df 컬럼 데이터 
        self.load_combobox(self.camera_df, 'cam_num', self.select_camera_box_video)
        self.load_combobox(self.camera_df, 'info_type', self.select_camera_type_box)
        self.load_combobox(self.camera_df, 'captured_date', self.captured_date_start)
        self.load_combobox(self.camera_df, 'captured_date', self.captured_date_end)
        
        self.load_combobox_harm(self.harmful_animal_df,'index_num',self.select_harmful_animal_index)
        
        self.search_button_animal.clicked.connect(self.animal_search)
        self.search_button_food.clicked.connect(self.food_search)
        # 시작 날짜 콤보박스 변경 시 종료 날짜 콤보박스 업데이트
        self.captured_date_start.currentIndexChanged.connect(self.update_captured_date_end)
        self.search_button_video.clicked.connect(self.camera_search)
        self.employee_register_button.clicked.connect(self.register_new_employee)
        self.add_harmful_animal_button.clicked.connect(self.register_new_harmful_animal)
        #self.delete_harmful_animal_button.clicked.connect(self.delete_harmful_animal)
        self.refresh_combo_box()
        self.select_harmful_animal_index.currentIndexChanged.connect(self.show_selected_animal_name)
        

    def show_selected_animal_name(self, index):
        selected_index = self.select_harmful_animal_index.itemText(index)
        if selected_index:
            # 해당 인덱스에 해당하는 animal_name을 찾아 라인에딧에 설정
            selected_animal_name = self.harmful_animal_df.loc[self.harmful_animal_df['index_num'] == int(selected_index), 'animal_name'].values
            if selected_animal_name:
                self.select_harmful_animal_name.setText(selected_animal_name[0])
            else:
                self.select_harmful_animal_name.clear()
                        
    def register_new_harmful_animal(self):
        animal_name= self.harmful_animal_name_edit.text()
        index_num = self.harmful_animal_index_box.currentText()
        self.data_manage.register_harmful_animal(index_num,animal_name)
        self.harmful_animal_df=self.data_manage.getHarmfulAnimal()
        self.load_data_to_table(self.harmful_animal_table, self.harmful_animal_df)
        self.refresh_combo_box()
    
    # def delete_harmful_animal(self):
    #     index_num= self.select_harmful_animal_index.currentText()  
    #     animal_name = self.select_harmful_animal_name.text()      
    #     self.data_manage.delete_harmful_animal(index_num,animal_name)
    #     self.harmful_animal_df=self.data_manage.getHarmfulAnimal()
    #     self.load_data_to_table(self.harmful_animal_table, self.harmful_animal_df)
    #     self.refresh_combo_box()

                
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

        # 다른 필터 적용
        for key, value in filters.items():
            if value != 'all':
                filtered_df = filtered_df[filtered_df[key].astype(str) == value]

        # 날짜 필터 적용
        if start_date_str != 'all':
            start_date = datetime.strptime(start_date_str, '%Y-%m-%d').date()
            filtered_df = filtered_df[filtered_df['captured_date'] >= start_date]
        if end_date_str != 'all':
            end_date = datetime.strptime(end_date_str, '%Y-%m-%d').date()
            filtered_df = filtered_df[filtered_df['captured_date'] <= end_date]

        # 결과를 테이블에 로드
        self.load_data_to_table(self.search_camera_table, filtered_df)
 
    
   
        
    def load_combobox(self, df, column_name, combobox):
        unique_values = np.sort(df[column_name].unique())
        combobox.addItem('all')
        combobox.addItems(map(str, unique_values))

    def load_combobox_harm(self, df, column_name, combobox):
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
            if robot.task_id != 0 : #할당이 안되어 있다 
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
                    return True
        return False
    
    def isNavArucoFinished(self):
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
                if self.isServiceCalled(robot.getTaskID()):
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
    
    def login_button_clicked(self):
        self.input_id = int(self.id_input.text())
        self.input_pw = int(self.pw_input.text())
        user_row = self.userdata_df[self.userdata_df['id'] == self.input_id]
        if not user_row.empty and user_row.iloc[0]['password'] == self.input_pw:
            self.stackedWidget.setCurrentIndex(Pages.PAGE_HOME.value)
            self.login_information_label.setText(f"ID: {self.input_id}님 접속중")
        else:
            self.login_result_label.setText('로그인에 실패하였습니다')
            
    def toolbox_changed(self):
        if self.toolBox.currentIndex() == 5:
            self.stackedWidget.setCurrentIndex(Pages.PAGE_LOG.value)
        elif self.toolBox.currentIndex() == 6:
            self.stackedWidget.setCurrentIndex(Pages.PAGE_SETTING.value)

    def home_page_button_clicked(self):
        self.stackedWidget.setCurrentIndex(Pages.PAGE_HOME.value)

    def monitor_barnpage_button_clicked(self):
        self.stackedWidget.setCurrentIndex(Pages.PAGE_MONITOR_BARN.value) 
    
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
        self.stackedWidget.setCurrentIndex(Pages.PAGE_REGISTER_ANIMAL.value)
    
    def search_animal_button_clicked(self):
        self.stackedWidget.setCurrentIndex(Pages.PAGE_SEARCH_ANIMAL.value)
    
    def choose_datamanager_foodpage_button_clicked(self):
        self.stackedWidget.setCurrentIndex(Pages.PAGE_CHOOSE_DATAMANAGER_FOOD.value)

    def register_food_button_clicked(self):
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
        
    def setupTableWidget(self, tableWidget):
        tableWidget.setRowCount(24)  # 행의 개수 (24행)
        tableWidget.setColumnCount(2)  # 열의 개수 (2열)
        tableWidget.horizontalHeader().setStretchLastSection(True)

        for hour in range(24):# 첫 번째 열에 시간 설정 (00:00부터 23:00까지)
            time_item = QTableWidgetItem(f"{hour:02d}:00")
            time_item.setFlags(time_item.flags() ^ Qt.ItemIsEditable)  
            tableWidget.setItem(hour, 0, time_item)
 
        for row in range(24):# 두 번째 열에 체크박스 추가
            checkbox_item = QTableWidgetItem()
            checkbox_item.setFlags(checkbox_item.flags() | Qt.ItemIsUserCheckable)
            checkbox_item.setCheckState(Qt.Unchecked)
            tableWidget.setItem(row, 1, checkbox_item)
        tableWidget.setHorizontalHeaderLabels(["Time", "Assigned"])
        
        
if __name__ == "__main__":
    app = QApplication(sys.argv)
    myWindows = WindowClass()
    myWindows.show()

    sys.exit(app.exec_())