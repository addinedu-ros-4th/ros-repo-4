import socket
import cv2
import pickle
import struct
from _thread import start_new_thread

HOST = '192.168.0.20'  # 서버 IP 주소
PORT = 5002
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect((HOST, PORT))

def recv_data(client_socket):
    while True:
        try:
            data = client_socket.recv(1024)
            if not data:
                break
            print("receive : ", repr(data.decode()))
        except Exception as e:
            print("Error receiving data:", e)
            break

start_new_thread(recv_data, (client_socket,))
print('>> Connect Server')

cap = cv2.VideoCapture(0)

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        data = pickle.dumps(frame)
        message_size = struct.pack("L", len(data))  # 데이터 길이를 패킹
        client_socket.sendall(message_size + data)
except Exception as e:
    print("Error sending data:", e)
finally:
    cap.release()
    client_socket.close()
