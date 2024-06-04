import cv2
import mediapipe as mp
import numpy as np

# MediaPipe Pose 모듈 초기화
mp_pose = mp.solutions.pose
pose = mp_pose.Pose(static_image_mode=False, min_detection_confidence=0.5, min_tracking_confidence=0.5)

# 카메라 캡처 시작
cap = cv2.VideoCapture(0)

def calculate_angle(a, b, c):
    # 점 a, b, c의 좌표
    a = np.array([a.x, a.y])
    b = np.array([b.x, b.y])
    c = np.array([c.x, c.y])
    
    # 벡터 계산
    ab = b - a
    bc = c - b
    
    # 벡터 간 각도 계산
    cosine_angle = np.dot(ab, bc) / (np.linalg.norm(ab) * np.linalg.norm(bc))
    angle = np.arccos(cosine_angle)
    
    return np.degrees(angle)

def detect_cow_pose(image):
    # 이미지를 RGB로 변환
    image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    
    # 포즈 추출
    results = pose.process(image_rgb)
    
    if not results.pose_landmarks:
        return "No cow detected"

    # 관절 포인트 추출
    landmarks = results.pose_landmarks.landmark

    # 필요한 관절 포인트의 인덱스
    LEFT_HIP = mp_pose.PoseLandmark.LEFT_HIP.value
    LEFT_KNEE = mp_pose.PoseLandmark.LEFT_KNEE.value
    LEFT_ANKLE = mp_pose.PoseLandmark.LEFT_ANKLE.value
    RIGHT_HIP = mp_pose.PoseLandmark.RIGHT_HIP.value
    RIGHT_KNEE = mp_pose.PoseLandmark.RIGHT_KNEE.value
    RIGHT_ANKLE = mp_pose.PoseLandmark.RIGHT_ANKLE.value

    # 왼쪽 다리 각도 계산
    left_hip = landmarks[LEFT_HIP]
    left_knee = landmarks[LEFT_KNEE]
    left_ankle = landmarks[LEFT_ANKLE]
    
    left_leg_angle = calculate_angle(left_hip, left_knee, left_ankle)

    # 오른쪽 다리 각도 계산
    right_hip = landmarks[RIGHT_HIP]
    right_knee = landmarks[RIGHT_KNEE]
    right_ankle = landmarks[RIGHT_ANKLE]
    
    right_leg_angle = calculate_angle(right_hip, right_knee, right_ankle)

    # 각도를 기준으로 자세 판별
    threshold_angle = 160  # 임계값, 각도가 160도 이상이면 서 있는 것으로 간주
    if left_leg_angle > threshold_angle and right_leg_angle > threshold_angle:
        return "Standing"
    else:
        return "Sitting"

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    pose_result = detect_cow_pose(frame)
    print(pose_result)

    # 결과 이미지 보여주기
    cv2.imshow('Cow Pose Detection', frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 캡처 해제 및 윈도우 닫기
cap.release()
cv2.destroyAllWindows()
