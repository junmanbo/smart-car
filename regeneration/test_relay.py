# 차를 조종하고 사진 저장하기

import cv2
import RPi.GPIO as GPIO
import time

# 실제 핀 정의
#PWM PIN
PWMA = 32
PWMB = 33

#GPIO PIN
AIN1 = 37
AIN2 = 31
BIN1 = 29
BIN2 = 22

relay_pin = 8

def motor_go(speed):
    L_Motor.ChangeDutyCycle(speed)
    GPIO.output(AIN2,False)
    GPIO.output(AIN1,True)
    R_Motor.ChangeDutyCycle(speed)
    GPIO.output(BIN2,False)
    GPIO.output(BIN1,True)

def motor_stop():
    L_Motor.ChangeDutyCycle(0)
    GPIO.output(AIN2,False)
    GPIO.output(AIN1,False)
    R_Motor.ChangeDutyCycle(0)
    GPIO.output(BIN2,False)
    GPIO.output(BIN1,False)

GPIO.setwarnings(False)
#  GPIO.setmode(GPIO.BCM)
GPIO.setmode(GPIO.BOARD)

# Relay Switch
GPIO.setup(relay_pin, GPIO.OUT)

# DC motor
GPIO.setup(AIN2,GPIO.OUT)
GPIO.setup(AIN1,GPIO.OUT)
GPIO.setup(PWMA,GPIO.OUT)

GPIO.setup(BIN1,GPIO.OUT)
GPIO.setup(BIN2,GPIO.OUT)
GPIO.setup(PWMB,GPIO.OUT)

L_Motor = GPIO.PWM(PWMA,50)
L_Motor.start(0)

R_Motor = GPIO.PWM(PWMB,50)
R_Motor.start(0)

speedSet = 50

def main():
    carState = 'stop'

    while True:
        command = int(input('주행 명령어 1. go 2. stop 3. quit: '))

        if command == 3:
            break
        elif command == 1:
            print('go')
            carState = 'go'
            GPIO.output(relay_pin, 1)
            motor_go(speedSet)

        elif command == 2:
            print('stop')
            carState = 'stop'
            GPIO.output(relay_pin, 0)
            motor_stop()

if __name__ == '__main__':
    main()
    GPIO.cleanup()
