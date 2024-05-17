import socket
import RPi.GPIO as GPIO
import time

# GPIO 핀 번호 설정 - 18
servo_pin = 18 

# GPIO 핀 모드 설정
GPIO.setmode(GPIO.BCM)
GPIO.setup(servo_pin, GPIO.OUT)

# PWM 객체 생성 - 주파수 50Hz
pwm = GPIO.PWM(servo_pin, 50) # 

# 서보 모터 각도 설정 함수
def set_angle(angle):
    duty = angle / 18 + 2
    GPIO.output(servo_pin, True)
    pwm.ChangeDutyCycle(duty)
    time.sleep(1)
    GPIO.output(servo_pin, False)
    pwm.ChangeDutyCycle(0)


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

        if reply:   
                tmp_str = str(reply)
                tmp_split = tmp_str.split(',')

                for val in tmp_split:
                        print(val)

                cleaned_str = tmp_split[1].replace("'", "")
                if cleaned_str == "1":
                        print("split 1")

                        pwm.start(0)
                        try:
                                set_angle(0)  # 0도로 회전 - 원하는 각도 입력
                        except KeyboardInterrupt:
                                pwm.stop()
                                GPIO.cleanup()

                elif cleaned_str == "0":
                        print("split 0")

                        pwm.start(0)
                        try:
                                set_angle(90)  # 0도로 회전 - 원하는 각도 입력
                        except KeyboardInterrupt:
                                pwm.stop()
                                GPIO.cleanup()
