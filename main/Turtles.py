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




#page 상수 정의
HOME_PAGE = 0
LOGIN_PAGE = 1
MONITOR_BARN_PAGE = 2
MONITOR_FACILITIES_PAGE = 3 
MONITOR_CAMERA_PAGE = 4
CONTROL_ROBOT_PAGE = 5
CONTROL_FACILITIES_PAGE = 6
ROBOTMANAGER_TASK_PAGE = 7
DATAMANAGER_ANIMAL_PAGE = 8
DATAMANAGER_FOOD_PAGE = 9
DATAMANAGER_VIDEO_PAGE = 10
DATAMANAGER_FACILITIES_PAGE = 11
SCHEDULE_ROBOT_PAGE = 12
SCHEDULE_FOOD_PAGE = 13
SCHEDULE_FACILITIES_PAGE = 14
LOG_PAGE = 15


server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
host = "192.168.0.86"
port = 3000
server_socket.bind((host, port))
server_socket.listen(5)

client_sockets = [server_socket]

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
    new_connection = pyqtSignal(object)

    def __init__(self):
        super().__init__()
        

    def run(self):
        while True:
            readable, _, _ = select.select(client_sockets, [], [])

            for sock in readable:
                if sock == server_socket:
                    # 새로운 클라이언트 연결
                    client_socket, _ = server_socket.accept()
                    client_sockets.append(client_socket)
                    print("새로운 클라이언트 연결")
                    print(len(client_sockets))

                else:
                    # 기존 클라이언트의 데이터 수신 및 처리
                    data = sock.recv(1024)
                    if not data:
                        # 클라이언트 연결 종료
                        client_sockets.remove(sock)
                        sock.close()
                    else:
                        print("받은 데이터:", data.decode())
                

    def stop(self):
        self.server_socket.close()
    
    


from_class = uic.loadUiType("Turtles.ui")[0]

class WindowClass(QMainWindow, from_class) :
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.setWindowTitle("Turtles : Herding Heroes")
        rp.init()

        #Thread
        self.tcpserver_thread = TcpServer(parent=self)
        self.count = 0
        self.tcpserverStart()

        #login 화면으로 초기화면 셋팅
        self.stackedWidget.setCurrentIndex(LOGIN_PAGE)

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

        #service call test
        self.service_call.clicked.connect(self.service_call_clicked)
        self.nav_to_station1_button.clicked.connect(self.nav_to_station1_button_clicked)
        self.nav_to_station2_button.clicked.connect(self.nav_to_station2_button_clicked)
        self.nav_to_foodtank1_button.clicked.connect(self.nav_to_foodtank1_button_clicked)
        self.nav_to_foodtank2_button.clicked.connect(self.nav_to_foodtank2_button_clicked)
        self.nav_to_barn_entrance_button.clicked.connect(self.nav_to_barn_entrance_button_clicked)
        self.nav_to_barn_exit_button.clicked.connect(self.nav_to_barn_exit_button_clicked)



        #tcp server thread  
        self.tcpserver_thread.update.connect(self.update_tcp_server_thread)

        #food trailer servo 버튼 연결
        self.foodtank_servo_open_button.clicked.connect(self.foodtank_servo_open_button_clicked)
        self.foodtank_servo_close_button.clicked.connect(self.foodtank_servo_close_button_clicked)
        self.foodtrailer_servo_open_button.clicked.connect(self.foodtrailer_servo_open_button_clicked)
        self.foodtrailer_servo_close_button.clicked.connect(self.foodtrailer_servo_close_button_clicked)

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
        client_sockets[1].send(data.encode("utf-8"))

    def send_to_rasp(self, data=""):
        #send
        # response = str(data)
        print(data)
        client_sockets[1].send(data.encode("utf-8"))

    def closeEvent(self,event):
        self.tcpserverStop()
        server_thread.stop()
        event.accept()

    def tcpserverStart(self):
        self.tcpserver_thread.start()

    def tcpserverStop(self):
        self.tcpserver_thread.stop()

    def update_tcp_server_thread(self):
        pass
        # # 서버 소켓 생성
        # server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # server_socket.bind((host, port))
        # server_socket.listen(5)

        # print(f"서버가 {host}:{port}에서 대기 중입니다...")

        # client_socket, client_address = server_socket.accept()
        # print(client_socket)
        # print(f"클라이언트 {client_address}가 연결되었습니다.")

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
        req.x = 0.6378898620605469
        req.y = -0.9446680545806885
        req.z =  -0.005340576171875
        
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
        req.x = 1.7415525913238525
        req.y = -0.3976095914840698
        req.z = 0.002471923828125

        
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
        self.stackedWidget.setCurrentIndex(LOGIN_PAGE)
    
    def login_button_clicked(self):
        self.stackedWidget.setCurrentIndex(HOME_PAGE)


    def toolbox_changed(self):
        if self.toolBox.currentIndex() == 5:
            self.stackedWidget.setCurrentIndex(LOG_PAGE)

    def home_page_button_clicked(self):
        self.stackedWidget.setCurrentIndex(HOME_PAGE)

    def monitor_barnpage_button_clicked(self):
        self.stackedWidget.setCurrentIndex(MONITOR_BARN_PAGE) 
    
    def monitor_facilitiespage_button_clicked(self):
        self.stackedWidget.setCurrentIndex(MONITOR_FACILITIES_PAGE)

    def monitor_camerapage_button_clicked(self):
        self.stackedWidget.setCurrentIndex(MONITOR_CAMERA_PAGE)
    
    def control_robotpage_button_clicked(self):
        self.stackedWidget.setCurrentIndex(CONTROL_ROBOT_PAGE)

    def control_facilitiespage_button_clicked(self):
        self.stackedWidget.setCurrentIndex(CONTROL_FACILITIES_PAGE)

    def robotmanager_taskpage_button_clicked(self):
        self.stackedWidget.setCurrentIndex(ROBOTMANAGER_TASK_PAGE)
    
    def datamanager_animalpage_button_clicked(self):
        self.stackedWidget.setCurrentIndex(DATAMANAGER_ANIMAL_PAGE)

    def datamanager_foodpage_button_clicked(self):
        self.stackedWidget.setCurrentIndex(DATAMANAGER_FOOD_PAGE)

    def datamanager_videopage_button_clicked(self):
        self.stackedWidget.setCurrentIndex(DATAMANAGER_VIDEO_PAGE)

    def datamanager_facilitiespage_button_clicked(self):
        self.stackedWidget.setCurrentIndex(DATAMANAGER_FACILITIES_PAGE)

    def schedule_robotpage_button_clicked(self):
        self.stackedWidget.setCurrentIndex(SCHEDULE_ROBOT_PAGE)

    def schedule_foodpage_button_clicked(self):
        self.stackedWidget.setCurrentIndex(SCHEDULE_FOOD_PAGE)

    def schedule_facilitiespage_button_clicked(self):
        self.stackedWidget.setCurrentIndex(SCHEDULE_FACILITIES_PAGE)

        
        




if __name__ == "__main__":
    app = QApplication(sys.argv)
    server_thread = ServerThread()
    server_thread.start()
    myWindows = WindowClass()
    myWindows.show()

    sys.exit(app.exec_())

