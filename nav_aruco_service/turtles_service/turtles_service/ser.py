import socket
import cv2
import pickle
import struct

# 서버 설정
TCP_IP = '192.168.0.20'
TCP_PORT = 5002

# 소켓 생성 및 바인드
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind((TCP_IP, TCP_PORT))
server_socket.listen(5)
print('서버가 시작되었습니다. 클라이언트를 기다립니다...')

conn, addr = server_socket.accept()
print('클라이언트가 연결되었습니다:', addr)

data = b""
payload_size = struct.calcsize("L")

try:
    while True:
        # 패킷 헤더를 수신하여 메시지 크기를 구합니다
        while len(data) < payload_size:
            packet = conn.recv(4*1024)
            if not packet: 
                break
            data += packet

        if len(data) < payload_size:
            break

        packed_msg_size = data[:payload_size]
        data = data[payload_size:]
        msg_size = struct.unpack("L", packed_msg_size)[0]

        # 메시지 크기만큼 데이터를 수신합니다
        while len(data) < msg_size:
            data += conn.recv(4*1024)

        frame_data = data[:msg_size]
        data = data[msg_size:]

        # 수신한 데이터를 역직렬화합니다
        frame = pickle.loads(frame_data)


finally:
    conn.close()
    server_socket.close()
    cv2.destroyAllWindows()
    print('서버가 종료되었습니다.')
