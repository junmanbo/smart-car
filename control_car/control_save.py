# 차를 조종하고 사진 저장하기

import cv2
import RPi.GPIO as GPIO
import time

# 실제 핀 정의
#PWM PIN
PWMA = 26  #37 pin
PWMB = 0   #27 pin

#GPIO PIN
AIN1 = 19  #37 pin
AIN2 = 13  #35 pin
BIN1 = 6   #31 pin
BIN2 = 5   #29 pin

# Servo PIN
servo_pin = 14 # 8 PIN

def motor_back(speed):
    L_Motor.ChangeDutyCycle(speed)
    GPIO.output(AIN2,False)
    GPIO.output(AIN1,True)
    R_Motor.ChangeDutyCycle(speed)
    GPIO.output(BIN2,False)
    GPIO.output(BIN1,True)

def motor_go(speed):
    L_Motor.ChangeDutyCycle(speed)
    GPIO.output(AIN2,False)
    GPIO.output(AIN1,True)
    R_Motor.ChangeDutyCycle(speed)
    GPIO.output(BIN2,False)
    GPIO.output(BIN1,True)

def motor_back(speed):
    L_Motor.ChangeDutyCycle(speed)
    GPIO.output(AIN2,True)
    GPIO.output(AIN1,False)
    R_Motor.ChangeDutyCycle(speed)
    GPIO.output(BIN2,True)
    GPIO.output(BIN1,False)

def motor_stop():
    L_Motor.ChangeDutyCycle(0)
    GPIO.output(AIN2,False)
    GPIO.output(AIN1,False)
    R_Motor.ChangeDutyCycle(0)
    GPIO.output(BIN2,False)
    GPIO.output(BIN1,False)

def motor_left(speed):
    L_Motor.ChangeDutyCycle(speed)
    GPIO.output(AIN2,False)
    GPIO.output(AIN1,True)
    R_Motor.ChangeDutyCycle(0)
    GPIO.output(BIN2,True)
    GPIO.output(BIN1,False)

def motor_right(speed):
    L_Motor.ChangeDutyCycle(0)
    GPIO.output(AIN2,True)
    GPIO.output(AIN1,False)
    R_Motor.ChangeDutyCycle(speed)
    GPIO.output(BIN2,False)
    GPIO.output(BIN1,True)

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

# Servo motor
GPIO.setup(servo_pin, GPIO.OUT)
s_pwm = GPIO.PWM(servo_pin, 50)
s_pwm.start(7.5)
time.sleep(0.2)
s_pwm.stop()

# DC motor
GPIO.setup(AIN2,GPIO.OUT)
GPIO.setup(AIN1,GPIO.OUT)
GPIO.setup(PWMA,GPIO.OUT)

GPIO.setup(BIN1,GPIO.OUT)
GPIO.setup(BIN2,GPIO.OUT)
GPIO.setup(PWMB,GPIO.OUT)

L_Motor = GPIO.PWM(PWMA,100)
L_Motor.start(0)

R_Motor = GPIO.PWM(PWMB,100)
R_Motor.start(0)

speedSet = 50

def main():
    camera = cv2.VideoCapture(-1)
    camera.set(3, 640)
    camera.set(4, 480)
    filepath = '/home/pi/AI_CAR/video/train'
    i = 0
    carState = 'stop'

    while ( camera.isOpened() ):

        keyValue = cv2.waitKey(10)

        if keyValue == ord('q'):
            break
        elif keyValue == 82:
            print('go')
            carState = 'go'
            motor_go(speedSet)
        elif keyValue == 84 and carState == 'stop':
            print('back')
            carState = 'back'
            motor_back(speedSet)
        elif keyValue == 84:
            print('stop')
            carState = 'stop'
            motor_stop()
        elif keyValue == 81:
            print('left')
            carState = 'left'
            s_pwm.start(7.5)
            time.sleep(0.2)
            s_pwm.ChangeDutyCycle(6.0)
            time.sleep(0.2)
            s_pwm.stop()
        elif keyValue == 83:
            print('right')
            carState = 'right'
            s_pwm.start(7.5)
            time.sleep(0.2)
            s_pwm.ChangeDutyCycle(9.0)
            time.sleep(0.2)
            s_pwm.stop()

        _, image = camera.read()
        image = cv2.flip(image, -1)
        cv2.imshow('Original', image)

        height, _, _ = image.shape
        save_image = image[int(height/2):,:,:]
        save_image = cv2.cvtColor(save_image, cv2.COLOR_BGR2YUV)
        save_image = cv2.GaussianBlur(save_image, (3,3), 0)
        save_image = cv2.resize(save_image, (200,66))
        cv2.imshow('Save', save_image)

        if carState == 'left':
            cv2.imwrite(f'{filepath}/L_{i:05d}.png', save_image)
            i += 1
        elif carState == 'right':
            cv2.imwrite(f'{filepath}/R_{i:05d}.png', save_image)
            i += 1
        elif carState == 'go':
            cv2.imwrite(f'{filepath}/S_{i:05d}.png', save_image)
            i += 1

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
    GPIO.cleanup()
