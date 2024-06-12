import sys
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QVBoxLayout
from PyQt5.QtGui import QPixmap, QPainter, QPen, QColor
from PyQt5.QtCore import Qt, QRect

class ImageLabelWidget(QWidget):
    def __init__(self, image_path):
        super().__init__()
        self.initUI(image_path)

    def initUI(self, image_path):
        self.image_label = QLabel()
        self.image_label.setAlignment(Qt.AlignCenter)

        # 이미지 로드 및 라벨에 표시
        pixmap = QPixmap(image_path)
        self.image_label.setPixmap(pixmap)

        # 레이아웃 설정
        layout = QVBoxLayout()
        layout.addWidget(self.image_label)
        self.setLayout(layout)

        self.image_width = pixmap.width()
        self.image_height = pixmap.height()

    def paintEvent(self, event):
        # 이미지 기준 좌표를 라벨 기준 좌표로 매핑
        label_x1, label_y1 = self.map_image_to_label(100, 100)
        label_x2, label_y2 = self.map_image_to_label(300, 300)

        # QPainter를 사용하여 라벨 위에 사각형 그리기
        painter = QPainter(self)
        painter.setPen(QPen(QColor(255, 0, 0), 2, Qt.SolidLine))
        painter.drawRect(QRect(label_x1, label_y1, label_x2 - label_x1, label_y2 - label_y1))

    def map_image_to_label(self, image_x, image_y):
        # 이미지 기준 좌표를 라벨 기준 좌표로 변환
        label_x = (image_x / self.image_width) * self.image_label.width()
        label_y = (image_y / self.image_height) * self.image_label.height()
        return int(label_x), int(label_y)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    widget = ImageLabelWidget('example_image.jpg')
    widget.show()
    sys.exit(app.exec_())
