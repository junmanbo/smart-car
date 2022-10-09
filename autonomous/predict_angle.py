# 차를 조종하고 사진 저장하기

import cv2
import RPi.GPIO as GPIO
import numpy as np
import tensorflow as tf
from tensorflow.keras.models import load_model
import time

# 실제 핀 정의
#PWM PIN
PWM_pin = 32

#GPIO PIN
DIR = 31

# Servo PIN
servo_pin = 7

def motor_go(speed):
    dc_motor.ChangeDutyCycle(speed)
    GPIO.output(DIR,True)

def motor_back(speed):
    dc_motor.ChangeDutyCycle(speed)
    GPIO.output(DIR,False)

def motor_stop():
    dc_motor.ChangeDutyCycle(0)
    GPIO.output(DIR,False)

def servo_control(degree):
    s_motor.ChangeDutyCycle(degree)
    time.sleep(0.5)

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)

# DC motor
GPIO.setup(DIR,GPIO.OUT)
GPIO.setup(PWM_pin,GPIO.OUT)

# Servo motor
GPIO.setup(servo_pin, GPIO.OUT)
s_motor = GPIO.PWM(servo_pin, 50)

# dc_motor 초기화
s_motor.start(6) # 정면
time.sleep(0.5)

dc_motor = GPIO.PWM(PWM_pin,50)
dc_motor.start(0)

speedSet = 50

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
    model_path = '/home/pi/Documents/smart-car/autonomous/lane_navigation_final.h5'
    model = load_model(model_path)

    i = 0
    carState = 'stop'

    while ( camera.isOpened() ):

        keyValue = cv2.waitKey(10)

        if keyValue == ord('q'):
            break

        _, image = camera.read()
        image = cv2.flip(image, -1)
        cv2.imshow('Original', image)

        preprocessed = img_preprocess(image)
        cv2.imshow('pre', preprocessed)

        X = np.asarray([preprocessed])
        steering_angle = int(model.predict(X)[0])
        print("predict angle: ", steering_angle)

        motor_go(speedSet)

        default_angle = 90

        diff_angle = default_angle - steering_angle
        servo_control(6 + (0.067 * diff_angle))

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
    GPIO.cleanup()
