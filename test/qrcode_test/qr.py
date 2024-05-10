import cv2
import pyzbar.pyzbar as pyzbar
import webbrowser

font = cv2.FONT_HERSHEY_SIMPLEX
is_web_opened = False

# 바코드 인식 및 테두리 설정
def read_frame(frame):
    global is_web_opened
    try:
        # 바코드 정보 decoding
        barcodes = pyzbar.decode(frame)
        # 바코드 정보가 여러개 이기 때문에 하나씩 해석
        for barcode in barcodes:
            # 바코드 rect정보
            x, y, w, h = barcode.rect
            # 바코드 데이터 디코딩
            barcode_info = barcode.data.decode('utf-8')
            # 인식한 바코드 사각형 표시
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
            # 인식한 바코드 사각형 위에 글자 삽입
            cv2.putText(frame, barcode_info, (x , y - 20), font, 0.5, (0, 0, 255), 1)
            if not is_web_opened:
                webbrowser.open(barcode_info)
                is_web_opened = True
            
        return frame
    except Exception as e:
        print(e)


def main():
    try:
        # 동영상 불러오기
        cap = cv2.VideoCapture(0)

        # 동영상 연결 되었는지 확인 및 영상 재생
        while cap.isOpened():
            # 실행 내역 및 프레임 가져오기
            ret, frame = cap.read()
            # 실행 내역이 true이면 프레임 출력
            if ret:
                # 바코드 인식
                frame = read_frame(frame)
                # 프레임 출력
                cv2.imshow("barcode reader", frame)
                
                # 사용자가 'r' 키를 누르면 다시 QR 코드를 찍을 수 있도록 is_web_opened 변수 재설정
                key = cv2.waitKey(1)
                if key == ord('r'):
                    global is_web_opened
                    is_web_opened = False
                    
                # 'ESC' 키를 누르면 프로그램 종료
                if key == 27:
                    break
            else:
                print("예외")
                break

    except Exception as e:
        print(e)
    finally:
        cap.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
