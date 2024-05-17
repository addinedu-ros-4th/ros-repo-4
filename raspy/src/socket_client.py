import socket

host = "192.168.0.86"
port = 3000

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((host,port))

#Lets loop awaiting for your input
while True:
        # s.send("rasp")
        reply = s.recv(1024)
        if reply == 'Terminate':
                break
        print(reply)