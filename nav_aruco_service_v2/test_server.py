import socket
import threading
import pickle
import struct
import cv2

TCP_IP = '192.168.1.104'  # 서버 IP 주소
TCP_PORT = 3001          # 서버 포트

clients = []

# 아르코 마커 설정
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
aruco_params = cv2.aruco.DetectorParameters_create()

def handle_client(client_socket, address):
    print(f'Connected by {address}')
    data = b""
    payload_size = struct.calcsize("L")
    clients.append(client_socket)

    while True:
        try:
            while len(data) < payload_size:
                packet = client_socket.recv(4096)
                if not packet:
                    break
                data += packet

            if len(data) < payload_size:
                continue

            packed_msg_size = data[:payload_size]
            data = data[payload_size:]
            msg_size = struct.unpack("L", packed_msg_size)[0]

            while len(data) < msg_size:
                data += client_socket.recv(4096)

            frame_data = data[:msg_size]
            data = data[msg_size:]

            frame = pickle.loads(frame_data)

            # 이미지를 디코딩하여 아르코 마커 탐지 및 ID 표시
            frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)

            if ids is not None:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                for i in range(len(ids)):
                    c = corners[i][0]
                    x = int(c[:, 0].mean())
                    y = int(c[:, 1].mean())
                    cv2.putText(frame, str(ids[i][0]), (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2, cv2.LINE_AA)


            # 이미지 데이터를 다른 클라이언트(클라이언트2)로 전송
            for client in clients:
                if client != client_socket:
                    try:
                        message_size = struct.pack("L", len(frame_data))
                        client.sendall(message_size + frame_data)
                    except Exception as e:
                        print(f'Error sending image to client {client.getpeername()}: {e}')
                        clients.remove(client)
                        client.close()

        except Exception as e:
            print(f'Error handling client {address}: {e}')
            break

    client_socket.close()
    clients.remove(client_socket)

def main():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((TCP_IP, TCP_PORT))
    server_socket.listen(5)
    print('Waiting for connections...')

    while True:
        conn, addr = server_socket.accept()
        client_thread = threading.Thread(target=handle_client, args=(conn, addr))
        client_thread.start()

if __name__ == "__main__":
    main()
