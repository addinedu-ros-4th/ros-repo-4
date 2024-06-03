import socket
import cv2
import pickle
import struct
import time

TCP_IP = '192.168.1.104'  # 서버 IP 주소
TCP_PORT = 3001          # 서버 포트

def send_images():
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((TCP_IP, TCP_PORT))
    print('Connected to server')

    cap = cv2.VideoCapture(0)
    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 70]  # 압축 품질 조정 (기본값: 90 -> 70)

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        # 이미지 해상도 줄이기 (기본 해상도에서 절반으로 줄임)
        frame = cv2.resize(frame, (640, 480))

        # 이미지를 압축하여 전송
        result, frame = cv2.imencode('.jpg', frame, encode_param)
        data = pickle.dumps(frame, 0)
        message_size = struct.pack("L", len(data))

        client_socket.sendall(message_size + data)
        print('Image sent to server')

        # 전송 주기 조정 (0.1초 대기)
        time.sleep(0.1)

    cap.release()
    client_socket.close()

if __name__ == "__main__":
    send_images()

