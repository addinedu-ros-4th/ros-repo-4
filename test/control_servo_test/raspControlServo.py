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

# 메인 함수 
if __name__ == "__main__":
    pwm.start(0)
    try:
        set_angle(0)  # 0도로 회전 - 원하는 각도 입력
    except KeyboardInterrupt:
        pwm.stop()
        GPIO.cleanup()
