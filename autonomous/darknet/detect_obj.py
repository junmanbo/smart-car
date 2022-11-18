# 차를 조종하고 사진 저장하기

import cv2
import RPi.GPIO as GPIO
import numpy as np
import tensorflow as tf
from tensorflow.keras.models import load_model
import time
from tensorflow.keras.preprocessing import image
from tensorflow import keras

from darknet import *

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
s_motor.start(5.6) # 정면
time.sleep(0.5)

dc_motor = GPIO.PWM(PWM_pin,50)
dc_motor.start(0)

speedSet = 1

def img_preprocess(img):
    height, _, _ = img.shape
    img = cv2.resize(img, (416, 416))
    #  img = img / 255
    return img

def darknet_helper(img, width, height, network, class_names):
    darknet_image = make_image(width, height, 3)
    #  img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    #  img_resized = cv2.resize(img_rgb, (width, height),interpolation=cv2.INTER_LINEAR)

    # get image ratios to convert bounding boxes to proper size
    #  img_height, img_width, _ = img.shape
    #  width_ratio = img_width/width
    #  height_ratio = img_height/height

    # run model on darknet style image to get detections
    #  copy_image_from_bytes(darknet_image, img_resized.tobytes())
    detections = detect_image(network, class_names, darknet_image)
    #  free_image(darknet_image)
    #  return detections, width_ratio, height_ratio
    return detections

def main():
    camera = cv2.VideoCapture(-1)
    camera.set(3, 320)
    camera.set(4, 240)

    # load in our YOLOv4 architecture network
    network, class_names, class_colors = load_network("cfg/yolov4-tiny-custom.cfg", "data/piDS.data", "backup/yolov4-tiny-custom_best.weights")
    width = network_width(network)
    height = network_height(network)

    while ( camera.isOpened() ):

        keyValue = cv2.waitKey(5)

        if keyValue == ord('q'):
            break

        _, img = camera.read()
        img = cv2.flip(img, -1)
        #  cv2.imshow('Original', img)

        preprocessed = img_preprocess(img)
        cv2.imshow('pre', preprocessed)

        # call our darknet helper on video frame
        #  detections, width_ratio, height_ratio = darknet_helper(preprocessed, network, class_names)
        detections = darknet_helper(preprocessed, width, height, network, class_names)
        print(detections)
        #  confidence = detections[1]
        #  print(f'label: {label}, confidence: {confidence}')

        #  if label == 'fake_stop':
        #      motor_stop()
        #      time.sleep(1)
        #  else:
        #      motor_go(speedSet)


    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
    GPIO.cleanup()
qq