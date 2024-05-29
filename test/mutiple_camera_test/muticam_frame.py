import cv2

class CameraManager:
    def __init__(self):
        self.available_index = []
        self.camera_list = []
        self.initialize_cameras()

    def initialize_cameras(self):
        for index in range(15):
            camera = cv2.VideoCapture(index)
            if camera.isOpened():
                self.available_index.append(index)
                camera.release()
        
        if len(self.available_index) > 0:
            for val in self.available_index:
                temp_cap = cv2.VideoCapture(val)
                self.camera_list.append(temp_cap)

        for idx, camera in enumerate(self.camera_list):
            if camera.isOpened():
                camera.set(cv2.CAP_PROP_FPS, 30)
            else:
                print("camera index", end="")
                print(idx)
                print("is not opened")

    def read_frames(self):
        frames = []
        for idx, camera in enumerate(self.camera_list):
            if camera.isOpened():
                retval, image = camera.read()
                if retval:
                    frames.append(image)
                else:
                    print(f"Failed to read frame from camera {idx}")
            else:
                print(f"Camera {idx} is not opened")
        return frames

# Example usage
camera_manager = CameraManager()
frames = camera_manager.read_frames()
for idx, frame in enumerate(frames):
    cv2.imshow(f'Camera {idx}', frame)

cv2.waitKey(0)
cv2.destroyAllWindows()
