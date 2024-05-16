import socket
import time
# 서버 설정
host = "192.168.0.86"  # 서버의 IP 주소 또는 도메인 이름
port = 3000       # 포트 번호

# 서버 소켓 생성
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind((host, port))
server_socket.listen(5)

print(f"서버가 {host}:{port}에서 대기 중입니다...")

client_socket, client_address = server_socket.accept()
print(f"클라이언트 {client_address}가 연결되었습니다.")

count = 0

while client_socket:
    # 클라이언트 연결 대기
    
    #receive
    data = client_socket.recv(1024).decode("utf-8")
    print(data)

    # #send
    # response = str(count)
    # print(response)
    # client_socket.send(response.encode("utf-8"))

    # count += 1
    
    time.sleep(1)
    
   
    

client_socket.close()