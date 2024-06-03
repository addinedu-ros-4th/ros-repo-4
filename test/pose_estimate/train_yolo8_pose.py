from ultralytics import YOLO
model = YOLO('yolov8n-pose.pt')

results = model.train(data='/home/dyjung/git_ws/ros-repo-4/test/pose_estimate/cow.v1i.yolov8/data.yaml', epochs=100, imgsz=640)