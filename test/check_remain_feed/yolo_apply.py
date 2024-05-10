#학습 코드 
from ultralytics import YOLO
model = YOLO("yolov8m.yaml")
model = YOLO() 
model.train(data='data.yaml', epochs=75) #data : data.yaml파일이 위치하는 경로

# 추론 & 바운딩 박스 크롭 저장 
!yolo task=detect mode=predict model=/home/addinedu/runs/detect/train31/weights/best.pt source=/home/addinedu/Desktop/yolo/qtest/* save=true save_txt=true save_crop=True
