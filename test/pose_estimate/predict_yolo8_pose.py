from ultralytics import YOLO

model = YOLO('/home/dyjung/git_ws/ros-repo-4/test/pose_estimate/last.pt')

results = model('/home/dyjung/Downloads/cow_stand_photo.jpg', stream=True)

print(results)

# Process results generator
for result in results:
    boxes = result.boxes  # Boxes object for bounding box outputs
    masks = result.masks  # Masks object for segmentation masks outputs
    keypoints = result.keypoints  # Keypoints object for pose outputs
    probs = result.probs  # Probs object for classification outputs
    obb = result.obb  # Oriented boxes object for OBB outputs
    result.show()  # display to screen
    result.save(filename="result.jpg")  # save to disk