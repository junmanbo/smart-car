# 차를 조종하고 사진 저장하기

import cv2
import RPi.GPIO as GPIO
import numpy as np
import tensorflow as tf
from tensorflow.keras.models import load_model
import time
from tensorflow.keras.applications.mobilenet_v2 import MobileNetV2, preprocess_input, decode_predictions
from tensorflow.keras.preprocessing import image
from tensorflow import keras

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

speedSet = 25

def img_preprocess(img):
    height, _, _ = img.shape
    img = img[int(height/2):,:,:]
    img = cv2.resize(img, (200, 66))
    img = img / 255
    return img

def main(model):
    camera = cv2.VideoCapture(-1)
    camera.set(3, 640)
    camera.set(4, 480)

    while ( camera.isOpened() ):

        keyValue = cv2.waitKey(5)

        if keyValue == ord('q'):
            break

        _, img = camera.read()
        img = cv2.flip(img, -1)
        cv2.imshow('Original', img)

        img_array = keras.preprocessing.image.img_to_array(img)
        img_array = tf.expand_dims(img_array, 0)

        predictions = model.predict(img_array)
        score = tf.nn.softmax(predictions[0])
        probability = np.max(score)*100
        print(probability)

        # 0: human 1: stop_sign
        if probability >= 70:
            motor_stop()
            time.sleep(1)
        else:
            motor_go(speedSet)

    cv2.destroyAllWindows()

if __name__ == '__main__':
    model_path = '/home/pi/Documents/smart-car/autonomous/object_detection_model.h5'
    model = load_model(model_path)
    main(model)
    GPIO.cleanup()
