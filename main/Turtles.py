import sys
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5 import uic, QtCore
from PyQt5.QtCore import QThread, pyqtSignal

import rclpy as rp
from turtles_service_msgs.srv import NavToPose
import time
import socket
import select 
import struct 
import pickle
import cv2
import pandas as pd 
from enum import Enum
from YamlFileManager import YamlFileManager
from datetime import datetime, timedelta


class Pages(Enum):
    #page 상수 정의
    PAGE_HOME = 0
    PAGE_LOGIN = 1
    PAGE_MONITOR_BARN = 2
    PAGE_MONITOR_FACILITIES = 3 
    PAGE_MONITOR_CAMERA = 4
    PAGE_CONTROL_ROBOT = 5
    PAGE_CONTROL_FACILITIES = 6
    PAGE_ROBOTMANAGER_TASK = 7
    PAGE_DATAMANAGER_ANIMAL = 8
    PAGE_DATAMANAGER_FOOD = 9
    PAGE_DATAMANAGER_VIDEO = 10
    PAGE_DATAMANAGER_FACILITIES = 11
    PAGE_SCHEDULE_ROBOT = 12
    PAGE_SCHEDULE_FOOD = 13
    PAGE_SCHEDULE_FACILITIES = 14
    PAGE_LOG = 15
    PAGE_SETTING = 16

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
    STATUS_ASSIGNED = 1

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

class Task:
    def __init__(self,id,schedule_type, room_num,task_time):
        self.task_id = id 
        self.task_type = TaskType.TASK_FOOD.value  #0: 배식 1:청소 2: 정리
        self.task_schedule_type = schedule_type #0: schedule, 1: registered
        self.assigned_robot_num = 0 #배정된 로봇 이름 
        self.task_result = 0 #0: not done , 1: done
        self.task_time = task_time #'2024-01-01 00:00:00' # 업무 할당 시간
        self.task_room_num = room_num # room은 1부터 시작 1,2,3,4
    
    def setTaskResult(self,task_result):
        self.task_result = task_result


    
class TcpServer(QThread):
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

        self.robot_list = []
        food_robot1 = RobotStatus(1,RobotType.ROBOT_FOOD.value)
        # food_robot2 = RobotStatus(2,RobotType.ROBOT_FOOD.value)
        
        self.robot_list.append(food_robot1)
        # self.robot_list.append(food_robot2)
        

        self.tcpserver_thread = TcpServer(parent=self)
        self.count = 0
        self.server_thread = None
        self.client_df = pd.DataFrame(columns=['IP', 'Port'])

        self.tcpserverStart()

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
        self.monitor_camerapage_button.clicked.connect(self.monitor_camerapage_button_clicked)

        self.control_robotpage_button.clicked.connect(self.control_robotpage_button_clicked)
        self.control_facilitiespage_button.clicked.connect(self.control_facilitiespage_button_clicked)
        
        self.robotmanager_taskpage_button.clicked.connect(self.robotmanager_taskpage_button_clicked)

        self.datamanager_animalpage_button.clicked.connect(self.datamanager_animalpage_button_clicked)
        self.datamanager_foodpage_button.clicked.connect(self.datamanager_foodpage_button_clicked)
        self.datamanager_videopage_button.clicked.connect(self.datamanager_videopage_button_clicked)
        self.datamanager_facilitiespage_button.clicked.connect(self.datamanager_facilitiespage_button_clicked)

        self.schedule_robotpage_button.clicked.connect(self.schedule_robotpage_button_clicked)
        self.schedule_foodpage_button.clicked.connect(self.schedule_foodpage_button_clicked)
        self.schedule_facilitiespage_button.clicked.connect(self.schedule_facilitiespage_button_clicked)

        self.toolBox.currentChanged.connect(self.toolbox_changed)

        #login logout 버튼 연결
        self.logout_button.clicked.connect(self.logout_button_clicked)
        self.login_button.clicked.connect(self.login_button_clicked)

    
        #tcp server thread  
        self.tcpserver_thread.update.connect(self.update_tcp_server_thread)

        #food trailer servo 버튼 연결
        self.foodtank_servo_open_button.clicked.connect(self.foodtank_servo_open_button_clicked)
        self.foodtank_servo_close_button.clicked.connect(self.foodtank_servo_close_button_clicked)
        self.foodtrailer_servo_open_button.clicked.connect(self.foodtrailer_servo_open_button_clicked)
        self.foodtrailer_servo_close_button.clicked.connect(self.foodtrailer_servo_close_button_clicked)

        #task add
        self.task_add_button.clicked.connect(self.task_add_button_clicked)
    

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
                        robot.setStatus(Status.STATUS_ASSIGNED.value)
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

    def robotStatusManager(self):

        for robot in self.robot_list:
            if robot.status == 0:
                return
            elif robot.status == 1:
                print("robot_num", end="")
                print(robot.robot_num)
                print("status: assigned")
                #print만 찍고 바로 다음 status로 넘겨줌
                robot.status += 1
            elif robot.status ==2:
                #amcl로 위치 확인
                #aruco 마커 확인
                print("robot_num", end="")
                print(robot.robot_num)
                print("status: station check")
                pass



    def update_tcp_server_thread(self):
        self.assignRobotTask()
        self.robotStatusManager()
        

    def task_add_button_clicked(self):
        #task 클래스로 객체 만들어서 list에 넣어주기
        current_datetime = datetime.now()
        modified_datetime = current_datetime.replace(hour=int(self.hour_edit.text()), minute=int(self.minutes_edit.text()), second=0, microsecond=0)
        self.task_id += 1
        temp_task = Task(self.task_id,TaskScheduleType.TASK_REGISTERED.value, self.room_number_edit.text(),modified_datetime)
        
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

        self.tcpserverStop()
        if self.server_thread:
            self.server_thread.stop()
        event.accept()

    def tcpserverStart(self):
        self.tcpserver_thread.start()

    def tcpserverStop(self):
        self.tcpserver_thread.stop()
        
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
        # 열의 수를 2로 고정(IP와 Port를 위한 열)
        self.client_table.setColumnCount(2)
        
        # 행의 수를 DataFrame의 길이(클라이언트 수)로 설정
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

        
        
    def service_call_clicked(self):
        pass
        # test_node = rp.create_node('client_test')
        
        # service_name = '/turtles'
        # cli = test_node.create_client(Target, service_name)
        # req = Target.Request()
        # req.target = "station1"
        

        # print(req)

        # while not cli.wait_for_service(timeout_sec=1.0):
        #     print("Waiting for service")

        # future = cli.call_async(req)

        # while not future.done():
        #     rp.spin_once(test_node)
        #     print(future.done(), future.result())

    def nav_to_station1_button_clicked(self):
        test_node = rp.create_node('client_test')
        
        service_name = '/nav_service'
        cli = test_node.create_client(NavToPose, service_name)
        req = NavToPose.Request()
        req.x = 0.007822726853191853
        req.y = -0.024536626413464546
        req.z = 0.002471923828125
        
        print(req)

        while not cli.wait_for_service(timeout_sec=1.0):
            print("Waiting for service")

        future = cli.call_async(req)

        while not future.done():
            rp.spin_once(test_node)
            print(future.done(), future.result())

    def nav_to_station2_button_clicked(self):
        test_node = rp.create_node('client_test')
        
        service_name = '/nav_service'
        cli = test_node.create_client(NavToPose, service_name)
        req = NavToPose.Request()
        req.x = 0.004148332867771387
        req.y = -0.8876231908798218
        req.z = 0.058197021484375
        
        print(req)

        while not cli.wait_for_service(timeout_sec=1.0):
            print("Waiting for service")

        future = cli.call_async(req)

        while not future.done():
            rp.spin_once(test_node)
            print(future.done(), future.result())

    def nav_to_foodtank1_button_clicked(self):
        test_node = rp.create_node('client_test')
        
        service_name = '/nav_service'
        cli = test_node.create_client(NavToPose, service_name)
        req = NavToPose.Request()
        req.x = 0.6322089433670044
        req.y = 0.03546354919672012
        req.z =  -0.001434326171875
        
        print(req)

        while not cli.wait_for_service(timeout_sec=1.0):
            print("Waiting for service")

        future = cli.call_async(req)

        while not future.done():
            rp.spin_once(test_node)
            print(future.done(), future.result())

    def nav_to_foodtank2_button_clicked(self):
        test_node = rp.create_node('client_test')
        
        service_name = '/nav_service'
        cli = test_node.create_client(NavToPose, service_name)
        req = NavToPose.Request()
        req.x = 1.0566225051879883
        req.y = 0.7861793637275696
        req.z = 0.002471923828125
        
        print(req)

        while not cli.wait_for_service(timeout_sec=1.0):
            print("Waiting for service")

        future = cli.call_async(req)

        while not future.done():
            rp.spin_once(test_node)
            print(future.done(), future.result())

    def nav_to_barn_entrance_button_clicked(self):
        test_node = rp.create_node('client_test')
        
        service_name = '/nav_service'
        cli = test_node.create_client(NavToPose, service_name)
        req = NavToPose.Request()
        req.x = 0.3123237192630768
        req.y = 0.7631283402442932
        req.z = -0.001434326171875

        
        print(req)

        while not cli.wait_for_service(timeout_sec=1.0):
            print("Waiting for service")

        future = cli.call_async(req)

        while not future.done():
            rp.spin_once(test_node)
            print(future.done(), future.result())

    def nav_to_barn_exit_button_clicked(self):
        test_node = rp.create_node('client_test')
        
        service_name = '/nav_service'
        cli = test_node.create_client(NavToPose, service_name)
        req = NavToPose.Request()
        req.x = 2.916684865951538
        req.y = -0.3893338143825531
        req.z = 0.002471923828125

        
        print(req)

        while not cli.wait_for_service(timeout_sec=1.0):
            print("Waiting for service")

        future = cli.call_async(req)

        while not future.done():
            rp.spin_once(test_node)
            print(future.done(), future.result())

    def logout_button_clicked(self):
        self.stackedWidget.setCurrentIndex(Pages.PAGE_LOGIN.value)
    
    def login_button_clicked(self):
        self.stackedWidget.setCurrentIndex(Pages.PAGE_HOME.value)


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
        self.stackedWidget.setCurrentIndex(Pages.PAGE_CONTROL_ROBOT.value)

    def control_facilitiespage_button_clicked(self):
        self.stackedWidget.setCurrentIndex(Pages.PAGE_CONTROL_FACILITIES.value)

    def robotmanager_taskpage_button_clicked(self):
        self.stackedWidget.setCurrentIndex(Pages.PAGE_ROBOTMANAGER_TASK.value)
        now = datetime.now()
        self.hour_edit.setText(str(now.hour))
        self.minutes_edit.setText(str(now.minute))


    
    def datamanager_animalpage_button_clicked(self):
        self.stackedWidget.setCurrentIndex(Pages.PAGE_DATAMANAGER_ANIMAL.value)

    def datamanager_foodpage_button_clicked(self):
        self.stackedWidget.setCurrentIndex(Pages.PAGE_DATAMANAGER_FOOD.value)

    def datamanager_videopage_button_clicked(self):
        self.stackedWidget.setCurrentIndex(Pages.PAGE_DATAMANAGER_VIDEO.value)

    def datamanager_facilitiespage_button_clicked(self):
        self.stackedWidget.setCurrentIndex(Pages.PAGE_DATAMANAGER_FACILITIES.value)

    def schedule_robotpage_button_clicked(self):
        self.stackedWidget.setCurrentIndex(Pages.PAGE_SCHEDULE_ROBOT.value)

    def schedule_foodpage_button_clicked(self):
        self.stackedWidget.setCurrentIndex(Pages.PAGE_SCHEDULE_FOOD.value)

    def schedule_facilitiespage_button_clicked(self):
        self.stackedWidget.setCurrentIndex(Pages.PAGE_SCHEDULE_FACILITIES.value)

        
        
if __name__ == "__main__":
    app = QApplication(sys.argv)
    myWindows = WindowClass()
    myWindows.show()

    sys.exit(app.exec_())