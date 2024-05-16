import sys
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5 import uic

import rclpy as rp
from turtlesim.srv import TeleportAbsolute #test를 위해서 
from turtles_msgs.srv import Target


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



from_class = uic.loadUiType("Turtles.ui")[0]

class WindowClass(QMainWindow, from_class) :
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.setWindowTitle("Turtles : Herding Heroes")

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


    def service_call_clicked(self):
        rp.init()
        test_node = rp.create_node('client_test')
        
        service_name = '/turtles'
        cli = test_node.create_client(Target, service_name)
        req = Target.Request()
        req.target = "station1"
        
        # service_name = '/turtle1/teleport_absolute'
        # cli = test_node.create_client(TeleportAbsolute, service_name)
        # req = TeleportAbsolute.Request()
        # req.x = 1.
        # req.y = 1.
        # req.theta = 3.14

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

    myWindows = WindowClass()

    myWindows.show()

    sys.exit(app.exec_())

