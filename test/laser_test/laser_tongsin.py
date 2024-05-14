from ultralytics import YOLO
import cv2
import re
from copy import deepcopy
from functools import lru_cache
from pathlib import Path
import serial
# Load a model
model = YOLO('yolov8n.pt')  # pretrained YOLOv8n model
model.predict(source="0", show=True, stream=True, verbose=False, classes=19)
DEVICE = '/dev/rfcomm3'
BAUD_RATE = 9600
s = serial.Serial(DEVICE, BAUD_RATE)
cap = cv2.VideoCapture(0)
while cap.isOpened():
    # Read a frame from the video
    success, frame = cap.read()
    if not success:
        break
    
    # Run batched inference on a list of images
    results = model(frame)  # return a list of Results objects
    
    # Process results list
    for result in results:
        verbose_output = result.verbose()
        
        # Check if 'cow' is in the verbose output
        if 'cow' in verbose_output:
            print(verbose_output)
            s.write(b"cow")
        else:
            s.write(b"end")
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
