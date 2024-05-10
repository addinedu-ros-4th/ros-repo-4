import cv2
from ultralytics import YOLO
model = YOLO("yolov8n.pt")

result = model.predict("cow2.jpeg")
