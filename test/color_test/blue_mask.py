import cv2
import numpy as np

# 카메라 캡처 시작
cap = cv2.VideoCapture(0)

while True:
    # 프레임 읽기
    ret, frame = cap.read()
    if not ret:
        break
    
    # BGR 이미지를 HSV로 변환
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # 파란색과 하늘색의 HSV 범위 정의
    lower_blue_cyan1 = np.array([90, 50, 50])
    upper_blue_cyan1 = np.array([130, 255, 255])
    lower_blue_cyan2 = np.array([70, 50, 50])
    upper_blue_cyan2 = np.array([90, 255, 255])
    
    # 두 범위의 마스크 생성
    mask1 = cv2.inRange(hsv, lower_blue_cyan1, upper_blue_cyan1)
    mask2 = cv2.inRange(hsv, lower_blue_cyan2, upper_blue_cyan2)
    
    # 두 마스크를 합쳐서 최종 마스크 생성
    mask = cv2.bitwise_or(mask1, mask2)
    
    # 원본 이미지와 마스크를 사용하여 파란색과 하늘색 부분 추출
    blue_cyan_only = cv2.bitwise_and(frame, frame, mask=mask)
    
    # 결과 이미지 보여주기
    cv2.imshow('Blue and Cyan Only', blue_cyan_only)
    
    # 'q' 키를 누르면 종료
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 캡처 해제 및 윈도우 닫기
cap.release()
cv2.destroyAllWindows()
