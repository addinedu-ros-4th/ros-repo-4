import socket
import cv2
import pickle
import struct

# 서버 설정
TCP_IP = '192.168.0.49'  # 클라이언트와 동일한 IP 주소 사용
TCP_PORT = 5002     # 클라이언트와 동일한 포트 사용

# 소켓 생성 및 바인드
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind((TCP_IP, TCP_PORT))
server_socket.listen(5)
print('서버가 시작되었습니다. 클라이언트를 기다립니다...')

conn, addr = server_socket.accept()
print('클라이언트가 연결되었습니다:', addr)

# 비디오 캡처
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    data = pickle.dumps(frame)
    message_size = struct.pack("L", len(data))  # 데이터 길이를 패킹
    conn.sendall(message_size + data)

cap.release()
conn.close()
server_socket.close()
