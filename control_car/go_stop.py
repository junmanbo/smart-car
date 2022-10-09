# 차를 조종하고 사진 저장하기

import RPi.GPIO as GPIO
import time

# 실제 핀 정의
#PWM PIN
PWMA = 32
#  PWMB = 33

#GPIO PIN
AIN1 = 37
AIN2 = 31
#  BIN1 = 29
#  BIN2 = 22

def motor_go(speed):
    Motor.ChangeDutyCycle(speed)
    #  GPIO.output(AIN1,False)
    GPIO.output(AIN2,True)

def motor_stop():
    Motor.ChangeDutyCycle(0)
    #  GPIO.output(AIN1,False)
    GPIO.output(AIN2,False)

GPIO.setwarnings(False)
#  GPIO.setmode(GPIO.BCM)
GPIO.setmode(GPIO.BOARD)

# DC motor
GPIO.setup(AIN2,GPIO.OUT)
#  GPIO.setup(AIN1,GPIO.OUT)
GPIO.setup(PWMA,GPIO.OUT)

#  GPIO.setup(BIN1,GPIO.OUT)
#  GPIO.setup(BIN2,GPIO.OUT)
#  GPIO.setup(PWMB,GPIO.OUT)

Motor = GPIO.PWM(PWMA,50)
Motor.start(0)

#  R_Motor = GPIO.PWM(PWMB,50)
#  R_Motor.start(0)

speedSet = 40

def main():
    try:
        while True:
            print('go')
            motor_go(speedSet)
            time.sleep(1.5)

            print('stop')
            carState = 'stop'
            motor_stop()
            time.sleep(1.5)
    except KeyboardInterrupt:
        motor_stop()
        GPIO.cleanup()

if __name__ == '__main__':
    main()
    GPIO.cleanup()
