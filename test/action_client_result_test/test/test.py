import sys
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5 import uic

import rclpy as rp

from test_package_msgs.srv import CountServiceMsg
from test_package_msgs.msg import ActionClientResult
from PyQt5.QtCore import QThread,pyqtSignal

import time


class WorkerThread(QThread):
    # 이 신호는 스레드에서 메인 스레드로 메시지를 보낼 때 사용됩니다.
    update_signal = pyqtSignal()
    def __init__(self):
        super().__init__()
        self.subscriber_node = rp.create_node('subscirb_node')
        self.subscriber = self.subscriber_node.create_subscription(ActionClientResult, '/pub_count',self.callback, 10)

    def run(self):
        time.sleep(1)  # 1초 대기
        # self.update_signal.emit()
        rp.spin(self.subscriber_node)

    def callback(self,data):
        print("callback-----")
        print(data.result)
        myWindows.putTexttoEdit(data.result)


from_class = uic.loadUiType("test.ui")[0]

class WindowClass(QMainWindow, from_class):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.setWindowTitle("test")
        rp.init()

        self.service_client_node = rp.create_node('test_service_client')
        self.service_name = '/count_service'
        self.cli = self.service_client_node.create_client(CountServiceMsg, self.service_name)
        self.req = CountServiceMsg.Request()
        
        self.result_sub = ""

        self.start_thread()

        self.pushButton.clicked.connect(self.button_clicked)

    

    def button_clicked(self):
        self.req.num = 10
        print(self.req)
        self.future = self.cli.call_async(self.req)

    def start_thread(self):
        self.thread = WorkerThread()
        self.thread.update_signal.connect(self.update)
        self.thread.start()

    def update(self):
        print("in here")
        # self.lineEdit.setText(message)
    
    def putTexttoEdit(self,tmp):
        self.lineEdit.setText(tmp)



if __name__ == "__main__":
    app = QApplication(sys.argv)
    myWindows = WindowClass()
    myWindows.show()
    sys.exit(app.exec_())