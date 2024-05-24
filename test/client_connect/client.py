import socket

def connect_to_server(host, port):
    # 서버에 연결하기 위한 소켓 생성
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    
    try:
        # 서버에 연결 시도
        client_socket.connect((host, port))
        print(f"{host}:{port} 서버에 연결되었습니다.")
    
        # 서버로부터의 메시지를 기다림 
        while True:
            message = client_socket.recv(1024)  # 1024 bytes를 받음
            if not message:
                # 서버로부터 빈 메시지를 받으면, 서버가 연결을 종료한 것으로 간주하고 반복문 종료 
                break
            print(f"서버로부터 받은 메시지: {message.decode()}")
        
        print("서버 연결이 종료되었습니다.")
    except Exception as e:
        print(f"서버에 연결하는 데 실패했습니다: {e}")
    finally:
        # 항상 연결을 종료합니다.
        if client_socket:
            client_socket.close()
        print("클라이언트 연결을 종료했습니다.")

if __name__ == "__main__":
    host = "192.168.0.138"  # 서버가 사용하는 ip
    port = 2000  # 서버가 사용하는 포트 번호
    connect_to_server(host, port)
