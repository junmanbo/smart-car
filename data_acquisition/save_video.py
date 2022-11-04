# 차를 조종하고 사진 저장하기

import cv2
import RPi.GPIO as GPIO
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

def servo_forward():
    s_motor.ChangeDutyCycle(6)
    time.sleep(0.5)

def servo_left():
    s_motor.ChangeDutyCycle(7.3)
    time.sleep(0.5)

def servo_right():
    s_motor.ChangeDutyCycle(4.7)
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

speedSet = 23

def main():
    camera = cv2.VideoCapture(-1)
    camera.set(3, 640)
    camera.set(4, 480)

    filepath = '/home/pi/Documents/smart-car/data_acquisition/tmp/video/'
    carState = 'stop'

    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter(filepath+'test.avi',fourcc, 20.0, (640,480))

    while ( camera.isOpened() ):

        keyValue = cv2.waitKey(10)

        if keyValue == ord('q'):
            break
        elif keyValue == 82 and carState == 'stop': # 위 방향키
            print('go')
            carState = 'go'
            motor_go(speedSet)
        elif keyValue == 84 and carState == 'stop': # 뒤 방향키
            print('back')
            carState = 'back'
            motor_back(speedSet)
        elif keyValue == 84 and carState != 'stop':
            print('stop')
            carState = 'stop'
            motor_stop()
        elif keyValue == 82:
            print('forward direction')
            servo_forward()
        elif keyValue == 81: # 왼쪽 방향키
            print('left')
            servo_left()
        elif keyValue == 83: # 오른쪽 방향키
            print('right')
            servo_right()

        _, image = camera.read()
        image = cv2.flip(image, -1)
        cv2.imshow('Original', image)
        out.write(image)

        #  height, _, _ = image.shape
        #  save_image = image[int(height/2):,:,:]
        #  save_image = cv2.cvtColor(save_image, cv2.COLOR_BGR2YUV)
        #  save_image = cv2.GaussianBlur(save_image, (3,3), 0)
        #  save_image = cv2.resize(save_image, (200,66))
        #  cv2.imshow('Save', save_image)
        #  cv2.imwrite(f'{filepath}/{i:05d}.png', save_image)

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
    GPIO.cleanup()
