import cv2
import numpy as np

# 카메라 캡처 객체를 생성합니다.
cap = cv2.VideoCapture(0)

# 카메라가 정상적으로 열렸는지 확인합니다.
if not cap.isOpened():
    print("카메라를 열 수 없습니다.")
    exit()

while True:
    # 프레임을 읽어옵니다.
    ret, frame = cap.read()
    
    # 프레임을 정상적으로 읽어왔는지 확인합니다.
    if not ret:
        print("프레임을 읽을 수 없습니다.")
        break
    
    # 프레임을 그레이스케일로 변환합니다.
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # 열화상 카메라 효과를 위해 colormap을 적용합니다.
    colormap = cv2.COLORMAP_JET  # 또는 cv2.COLORMAP_HOT
    thermal_frame = cv2.applyColorMap(gray_frame, colormap)
    
    # 결과 프레임을 화면에 표시합니다.
    cv2.imshow('Thermal Camera Effect', thermal_frame)
    
    # 'q' 키를 누르면 종료합니다.
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 캡처 객체와 창을 해제합니다.
cap.release()
cv2.destroyAllWindows()
