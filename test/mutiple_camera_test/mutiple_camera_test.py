import cv2

def check_camera(index):
    cap = cv2.VideoCapture(index)
    if not cap.isOpened():
        return False
    cap.release()
    return True

# 확인할 최대 카메라 수 (필요에 따라 조정)
max_tested_cameras = 50

# 사용 가능한 카메라 장치 목록
available_cameras = []

# 카메라 장치 확인
for i in range(max_tested_cameras):
    if check_camera(i):
        available_cameras.append(i)

if not available_cameras:
    print("No cameras found")
else:
    print(f"Found cameras: {available_cameras}")

    for index in available_cameras:
        cap = cv2.VideoCapture(index)

        if not cap.isOpened():
            print(f"Camera {index} cannot be opened")
            continue

        print(f"Displaying Camera {index}")

        while True:
            ret, frame = cap.read()
            if not ret:
                print(f"Cannot receive frame from Camera {index}")
                break

            cv2.imshow(f'Camera {index}', frame)

            if cv2.waitKey(1) == ord('q'):
                break

        # 비디오 캡처 객체 해제
        cap.release()
        cv2.destroyAllWindows()

    cv2.destroyAllWindows()
