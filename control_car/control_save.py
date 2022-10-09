# 차를 조종하고 사진 저장하기

import cv2
import RPi.GPIO as GPIO
import time

# 실제 핀 정의
#PWM PIN
PWMA = 32
PWMB = 33

#GPIO PIN
#  AIN1 = 37
AIN2 = 31
#  BIN1 = 29
#  BIN2 = 22

# Servo PIN
servo_pin = 7

def motor_go(speed):
    L_Motor.ChangeDutyCycle(speed)
    GPIO.output(AIN1,False)
    GPIO.output(AIN2,True)
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
    #  GPIO.output(AIN1,False)
    #  R_Motor.ChangeDutyCycle(0)
    #  GPIO.output(BIN2,False)
    #  GPIO.output(BIN1,False)

def servo_forward():
    s_pwm.ChangeDutyCycle(7.5)
    time.sleep(0.5)

def servo_left():
    s_pwm.ChangeDutyCycle(9)
    time.sleep(0.5)

def servo_right():
    s_pwm.ChangeDutyCycle(6)
    time.sleep(0.5)

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
#  GPIO.setup(AIN1,GPIO.OUT)
GPIO.setup(PWMA,GPIO.OUT)

#  GPIO.setup(BIN1,GPIO.OUT)
#  GPIO.setup(BIN2,GPIO.OUT)
#  GPIO.setup(PWMB,GPIO.OUT)

L_Motor = GPIO.PWM(PWMA,50)
L_Motor.start(0)

#  R_Motor = GPIO.PWM(PWMB,50)
#  R_Motor.start(0)

speedSet = 50

def main():
    camera = cv2.VideoCapture(-1)
    camera.set(3, 640)
    camera.set(4, 480)
    filepath = '/home/pi/Documents/smart-car/video/train'
    i = 0
    carState = 'stop'

    while ( camera.isOpened() ):

        keyValue = cv2.waitKey(10)

        if keyValue == ord('q'):
            break
        elif keyValue == 82 and carState == 'stop':
            print('go')
            carState = 'go'
            motor_go(speedSet)
        elif keyValue == 84 and carState == 'stop':
            print('back')
            carState = 'back'
            motor_back(speedSet)
        elif keyValue == 84 and (carState == 'go' or carState == 'back'):
            print('stop')
            carState = 'stop'
            motor_stop()
        elif keyValue == 82 and carState == 'go': # forward arrow key
            print('forward direction')
            servo_forward()
        elif keyValue == 81 and carState == 'go': # left arrow key
            print('left')
            servo_left()
        elif keyValue == 83 and carState == 'go': # right arrow key
            print('right')
            servo_right()

        _, image = camera.read()
        image = cv2.flip(image, -1)
        cv2.imshow('Original', image)

        height, _, _ = image.shape
        save_image = image[int(height/2):,:,:]
        save_image = cv2.cvtColor(save_image, cv2.COLOR_BGR2YUV)
        save_image = cv2.GaussianBlur(save_image, (3,3), 0)
        save_image = cv2.resize(save_image, (200,66))
        cv2.imshow('Save', save_image)

        cv2.imwrite(f'{filepath}/{i:05d}.png', save_image)
        i += 1

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
    GPIO.cleanup()
