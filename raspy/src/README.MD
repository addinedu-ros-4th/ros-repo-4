Raspberry Pi 에서 socket client로 연결하기 
---

1. ifconfig로 서버와 연결할 수 있는지 확인 
![Screenshot from 2024-05-17 14-25-56](https://github.com/addinedu-ros-4th/ros-repo-4/assets/137265648/611945a7-a398-4afd-af6b-ae91f3476ce6)

위 사진은  서버 ip 주소 
255.255.255.*  <- net mask가 255.255.255.0 이므로 맨 마지막 자리 빼고는 같아야 함

2. raspberry Pi에 socket 설치 
  sudo apt-get install socket
설치 문제 있으면 sudo apt update 해보기!

3. Rpi.GPIO 패키지 설치 
  sudo apt-get install python3-dev
  sudo apt-get install python3-rpi.gpio

* 인터넷에 찾아보면 python3 대신 python 이 들어간 명령어를 쓰던데 그러면 모듈 import에서 에러남 

4. 소스 참고하여 라즈베리 파이에 서보모터 연결하기 


