# 차를 조종하고 사진 저장하기

import cv2
import RPi.GPIO as GPIO
import time
import logging
import numpy as np
import math
import datetime
import sys

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
    s_motor.ChangeDutyCycle(7.5)
    time.sleep(0.5)

def servo_right():
    s_motor.ChangeDutyCycle(4.5)
    time.sleep(0.5)

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

def detect_lane(frame):
    logging.debug('detecting lane lines...')

    edges = detect_edges(frame)

    cropped_edges = region_of_interest(edges)

    line_segments = detect_line_segments(cropped_edges)
    line_segment_image = display_lines(frame, line_segments)

    lane_lines = average_slope_intercept(frame, line_segments)
    lane_lines_image = display_lines(frame, lane_lines)

    return lane_lines, lane_lines_image


def detect_edges(frame):
    # filter for blue lane lines
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_blue = np.array([30, 40, 0])
    upper_blue = np.array([150, 255, 255])
    #  lower_blue = np.array([20, 30, 150])
    #  upper_blue = np.array([150, 150, 150])
    mask = cv2.inRange(hsv, lower_blue, upper_blue)

    # detect edges
    edges = cv2.Canny(mask, 200, 400)

    return edges

def region_of_interest(canny):
    height, width = canny.shape
    mask = np.zeros_like(canny)

    # only focus bottom half of the screen

    polygon = np.array([[
        (0, height * 1 / 2),
        (width, height * 1 / 2),
        (width, height),
        (0, height),
    ]], np.int32)

    cv2.fillPoly(mask, polygon, 255)
    masked_image = cv2.bitwise_and(canny, mask)
    return masked_image


def detect_line_segments(cropped_edges):
    # tuning min_threshold, minLineLength, maxLineGap is a trial and error process by hand
    rho = 1  # precision in pixel, i.e. 1 pixel
    angle = np.pi / 180  # degree in radian, i.e. 1 degree
    min_threshold = 10  # minimal of votes
    line_segments = cv2.HoughLinesP(cropped_edges, rho, angle, min_threshold, np.array([]), minLineLength=8,
                                    maxLineGap=4)

    if line_segments is not None:
        for line_segment in line_segments:
            logging.debug('detected line_segment:')
            logging.debug("%s of length %s" % (line_segment, length_of_line_segment(line_segment[0])))

    return line_segments


def average_slope_intercept(frame, line_segments):
    """
    This function combines line segments into one or two lane lines
    If all line slopes are < 0: then we only have detected left lane
    If all line slopes are > 0: then we only have detected right lane
    """
    lane_lines = []
    if line_segments is None:
        logging.info('No line_segment segments detected')
        return lane_lines

    height, width, _ = frame.shape
    left_fit = []
    right_fit = []

    boundary = 1/3
    left_region_boundary = width * (1 - boundary)  # left lane line segment should be on left 2/3 of the screen
    right_region_boundary = width * boundary # right lane line segment should be on left 2/3 of the screen

    for line_segment in line_segments:
        for x1, y1, x2, y2 in line_segment:
            if x1 == x2:
                logging.info('skipping vertical line segment (slope=inf): %s' % line_segment)
                continue
            fit = np.polyfit((x1, x2), (y1, y2), 1)
            slope = fit[0]
            intercept = fit[1]
            if slope < 0:
                if x1 < left_region_boundary and x2 < left_region_boundary:
                    left_fit.append((slope, intercept))
            else:
                if x1 > right_region_boundary and x2 > right_region_boundary:
                    right_fit.append((slope, intercept))

    left_fit_average = np.average(left_fit, axis=0)
    if len(left_fit) > 0:
        lane_lines.append(make_points(frame, left_fit_average))

    right_fit_average = np.average(right_fit, axis=0)
    if len(right_fit) > 0:
        lane_lines.append(make_points(frame, right_fit_average))

    logging.debug('lane lines: %s' % lane_lines)  # [[[316, 720, 484, 432]], [[1009, 720, 718, 432]]]

    return lane_lines


def compute_steering_angle(frame, lane_lines):
    """ Find the steering angle based on lane line coordinate
        We assume that camera is calibrated to point to dead center
    """
    if len(lane_lines) == 0:
        logging.info('No lane lines detected, do nothing')
        return -90

    height, width, _ = frame.shape
    if len(lane_lines) == 1:
        logging.debug('Only detected one lane line, just follow it. %s' % lane_lines[0])
        x1, _, x2, _ = lane_lines[0][0]
        x_offset = x2 - x1
    else:
        _, _, left_x2, _ = lane_lines[0][0]
        _, _, right_x2, _ = lane_lines[1][0]
        camera_mid_offset_percent = 0.02 # 0.0 means car pointing to center, -0.03: car is centered to left, +0.03 means car pointing to right
        mid = int(width / 2 * (1 + camera_mid_offset_percent))
        x_offset = (left_x2 + right_x2) / 2 - mid

    # find the steering angle, which is angle between navigation direction to end of center line
    y_offset = int(height / 2)

    angle_to_mid_radian = math.atan(x_offset / y_offset)  # angle (in radian) to center vertical line
    angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi)  # angle (in degrees) to center vertical line
    steering_angle = angle_to_mid_deg + 90  # this is the steering angle needed by picar front wheel

    logging.debug('new steering angle: %s' % steering_angle)
    return steering_angle


def stabilize_steering_angle(curr_steering_angle, new_steering_angle, num_of_lane_lines, max_angle_deviation_two_lines=5, max_angle_deviation_one_lane=1):
    """
    Using last steering angle to stabilize the steering angle
    This can be improved to use last N angles, etc
    if new angle is too different from current angle, only turn by max_angle_deviation degrees
    """
    if num_of_lane_lines == 2 :
        # if both lane lines detected, then we can deviate more
        max_angle_deviation = max_angle_deviation_two_lines
    else :
        # if only one lane detected, don't deviate too much
        max_angle_deviation = max_angle_deviation_one_lane

    angle_deviation = new_steering_angle - curr_steering_angle
    if abs(angle_deviation) > max_angle_deviation:
        stabilized_steering_angle = int(curr_steering_angle
                                        + max_angle_deviation * angle_deviation / abs(angle_deviation))
    else:
        stabilized_steering_angle = new_steering_angle
    logging.info('Proposed angle: %s, stabilized angle: %s' % (new_steering_angle, stabilized_steering_angle))
    return stabilized_steering_angle

def display_lines(frame, lines, line_color=(0, 255, 0), line_width=10):
    line_image = np.zeros_like(frame)
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(line_image, (x1, y1), (x2, y2), line_color, line_width)
    line_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)
    return line_image

def main():
    camera = cv2.VideoCapture(-1)
    camera.set(3, 640)
    camera.set(4, 480)

    carState = 'stop'

    while ( camera.isOpened() ):

        keyValue = cv2.waitKey(10)
        motor_go(speedSet)

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

        default_angle = 90

        lane_lines, frame = detect_lane(image)
        steering_angle = compute_steering_angle(frame, lane_lines)
        final_angle = stabilize_steering_angle(default_angle, steering_angle, len(lane_lines))

        print(f'lane lines: {lane_lines} steering angle: {steering_angle}')


        diff_angle = default_angle - final_angle
        servo_duty = 6 + (0.067*diff_angle)
        print(f'servo duty cycle: {servo_duty}')

        servo_control(servo_duty)

    cv2.destroyAllWindows()

if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)
    main()
    GPIO.cleanup()
