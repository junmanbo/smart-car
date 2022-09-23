# 차를 조종하고 사진 저장하기

import cv2
import RPi.GPIO as GPIO
import numpy as np
import tensorflow as tf
from tensorflow.keras.models import load_model
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

# Servo PIN
servo_pin = 8

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
#  GPIO.setmode(GPIO.BCM)
GPIO.setmode(GPIO.BOARD)

# Servo motor
GPIO.setup(servo_pin, GPIO.OUT)
s_pwm = GPIO.PWM(servo_pin, 50)

s_pwm.start(7.5) # 정면
time.sleep(0.5)

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

speedSet = 80

def img_preprocess(image):
    height, _, _ = image.shape
    image = image[int(height/2):,:,:]
    image = cv2.cvtColor(image, cv2.COLOR_BGR2YUV)
    image = cv2.GaussianBlur(image, (3,3), 0)
    image = cv2.resize(image, (200, 66))
    image = image / 255
    return image

def main():
    camera = cv2.VideoCapture(-1)
    camera.set(3, 640)
    camera.set(4, 480)
    model_path = '/home/pi/Documents/smart-car/lane_navigation_final.h5'
    model = load_model(model_path)

    i = 0
    carState = 'stop'

    while ( camera.isOpened() ):

        keyValue = cv2.waitKey(10)

        if keyValue == ord('q'):
            break
        elif keyValue == 82:
            print('go')
            carState = 'go'
        elif keyValue == 84:
            print('stop')
            carState = 'stop'
        elif keyValue == 81:
            print('left')
            carState = 'left'
        elif keyValue == 83:
            print('right')
            carState = 'right'

        _, image = camera.read()
        image = cv2.flip(image, -1)
        cv2.imshow('Original', image)

        preprocessed = img_preprocess(image)
        cv2.imshow('pre', preprocessed)

        X = np.asarray([preprocessed])
        steering_angle = int(model.predict(X)[0])
        print("predict angle: ", steering_angle)

        if carState == "go":
            if 480 <= steering_angle <= 500:
                print("go")
                motor_go(speedSet)
            elif 600 >= steering_angle > 500:
                print("right")
                s_pwm.ChangeDutyCycle(6)
                time.sleep(0.5)
            elif 100 <= steering_angle < 480:
                print("left")
                s_pwm.ChangeDutyCycle(9.5)
                time.sleep(0.5)
            elif steering_angle < 100  or 600 < steering_angle:
                carState = 'stop'
                motor_stop()
        elif carState == "stop":
            motor_stop()

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
    GPIO.cleanup()
