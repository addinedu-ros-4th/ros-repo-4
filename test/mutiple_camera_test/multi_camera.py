import sys
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5 import uic
import cv2
import time
from PyQt5.QtCore import QThread
from PyQt5 import QtCore
import datetime

# Found cameras: [0, 2, 4, 6]

class Camera(QThread):
    update = QtCore.pyqtSignal()

    def __init__(self, sec =0, parent = None):
        super().__init__()
        self.main = parent
        self.running = True

    def run(self):
        count =0
        while self.running == True:
            self.update.emit()
            time.sleep(0.1)

    def stop(self):
        self.running = False


from_class = uic.loadUiType("multi_camera.ui")[0]

class WindowClass(QMainWindow, from_class) :
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.setWindowTitle("Hello, Qt!")

        self.camera = Camera(self)
        self.camera.daemon = True

        self.pixmap = QPixmap()
        
        self.camera.update.connect(self.updateCamera)

        self.comboBox.currentIndexChanged.connect(self.combochanged)
        self.cam_num = 0
        self.cameraStart()

    def combochanged(self):
        self.cam_num = int(self.comboBox.currentText())
        print(self.cam_num)



    def updateCamera(self):

        retval, image = self.video1.read()
        retval2, image2 = self.video2.read()
        retval3, image3 = self.video3.read()
        retval4, image4 = self.video4.read()

        print("retval2")
        print(retval2)
        print("retval3")
        print(retval3)
        print("retval4")
        print(retval4)

        if retval:
            if self.cam_num == 2:
                self.image = cv2.cvtColor(image2, cv2.COLOR_BGR2RGB)
            elif self.cam_num == 3:
                self.image = cv2.cvtColor(image3, cv2.COLOR_BGR2RGB)
            elif self.cam_num == 4:
                self.image = cv2.cvtColor(image4, cv2.COLOR_BGR2RGB)
            else:
                self.image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)


            h,w,c = self.image.shape
            qimage = QImage(self.image.data, w, h, w*c, QImage.Format_RGB888)

            self.pixmap = self.pixmap.fromImage(qimage)
            self.pixmap = self.pixmap.scaled(self.label.width(), self.label.height())

            self.label.setPixmap(self.pixmap)
        else:
            print(retval)
            print(f"Camera cannot be opened")

    

    def cameraStart(self):
        self.camera.running = True
        self.camera.start()
        self.video1 = cv2.VideoCapture(0)
        self.video2 = cv2.VideoCapture(2)
        self.video3 = cv2.VideoCapture(4)
        self.video4 = cv2.VideoCapture(6)



    def cameraStop(self):
        self.camera.running = False
        self.video1.release
        self.video2.release
        self.video3.release
        self.video4.release


if __name__ == "__main__":
    app = QApplication(sys.argv)
    myWindows = WindowClass()
    myWindows.show()
    sys.exit(app.exec_())

        
