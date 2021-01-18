
#!/usr/bin/env python3
##jwc o #!/usr/bin/env python

# Key Notes
# 
# jwc 2020-0519 Convert from Crickit_Adafruit to RoboHat_4tronix
# jwc 2020-0519 Use 'gpiozero' for noise-free servo-control
# jwc           Add 'robohat.init()'
# jwc           Make sure 'servod' copied in from 'RoboHat' dir
# jwc           Using 'robohat's' servo cause pan to jitter/cool and tilt to get hot
# jwc 2020-1213 Port in '~/01-Jwc/2020-1205-0640-RpiRover1-DayDevelops/RpiRover1-master' for Slider Ux, db directory
# jwc 2020-1223 AiCam Gen 2.0: StreamVideoToWebBrowser-AdrianRosebrock
#               * KEYESTUDIO Fisheye Wide Angle Lens 5MP 1080p OV5647 Sensor Module Supports Night Vision: 130-degrees
#               * Smraza 4 Camera Module 5 Megapixels 1080p OV5647 Sensor Adjustable Focus Wide Angle Fish-Eye Camera: 160-degrees
# jwc 2020-1230 AiCam Gen 2.1: 11j-OpenCv-DetectArucoMarkers-FreeBlog/opencv-detect-aruco 


from importlib import import_module
import os
import sys
import signal
import threading
import time
import json
from flask import Flask, render_template, request, Response
# jwc n from gevent.wsgi import WSGIServer
# jwc from gevent.pywsgi import WSGIServer
from waitress import serve


## jwc replace w/ PiUpTimeUps: # * BatteryUps: GeekPi/52pi.com: EP-0118
## jwc replace w/ PiUpTimeUps: # * https://wiki.52pi.com/index.php/UPS_(With_RTC_%26_Coulometer)_For_Raspberry_Pi_SKU:_EP-0118
## jwc replace w/ PiUpTimeUps: #
## jwc replace w/ PiUpTimeUps: from ina219 import INA219
## jwc replace w/ PiUpTimeUps: from ina219 import DeviceRangeError
## jwc replace w/ PiUpTimeUps: resistor_Shunt_OHM_GLOBAL = 0.05
## jwc replace w/ PiUpTimeUps: 
## jwc replace w/ PiUpTimeUps: # Define method to read information from coulometer.
## jwc replace w/ PiUpTimeUps: batteryUps_ClObj_Global = INA219(resistor_Shunt_OHM_GLOBAL)
## jwc replace w/ PiUpTimeUps: batteryUps_ClObj_Global.configure()

##jwc replace w/ PiUpTimeUps: def batteryUps_Read_Fn(config_In):
##jwc replace w/ PiUpTimeUps:     global batteryUps_ClObj_Global
##jwc replace w/ PiUpTimeUps: 
##jwc replace w/ PiUpTimeUps:     config_In.batteryUps_Volts_Input_V = batteryUps_ClObj_Global.voltage()
##jwc replace w/ PiUpTimeUps:     ##jwc y print("*** DEBUG: batteryUps_Volts_Input_V: %.3f V" % config_In.batteryUps_Volts_Input_V)
##jwc replace w/ PiUpTimeUps:     print(f"*** DEBUG: batteryUps_Volts_Input_V: {config_In.batteryUps_Volts_Input_V:.2f} V", end='')
##jwc replace w/ PiUpTimeUps:     try:
##jwc replace w/ PiUpTimeUps:         config_In.batteryUps_Volts_Output_V = batteryUps_ClObj_Global.shunt_voltage()
##jwc replace w/ PiUpTimeUps:         ##jwc y print("*** DEBUG: batteryUps_Volts_Output_V: %.3f mV" % config_In.batteryUps_Volts_Output_V)
##jwc replace w/ PiUpTimeUps:         print(f" // batteryUps_Volts_Output_V: {config_In.batteryUps_Volts_Output_V:.2f} mV", end='')
##jwc replace w/ PiUpTimeUps: 
##jwc replace w/ PiUpTimeUps:         config_In.batteryUps_Temp_C = batteryUps_ClObj_Global.current()
##jwc replace w/ PiUpTimeUps:         ##jwc y print("*** DEBUG: batteryUps_Temp_C: %.3f mA" % config_In.batteryUps_Temp_C)
##jwc replace w/ PiUpTimeUps:         print(f" // batteryUps_Temp_C: {config_In.batteryUps_Temp_C:.2f} mA", end='')
##jwc replace w/ PiUpTimeUps: 
##jwc replace w/ PiUpTimeUps:         config_In.batteryUps_Temp_F = batteryUps_ClObj_Global.power()
##jwc replace w/ PiUpTimeUps:         ##jwc y print("*** DEBUG: batteryUps_Temp_F: %.3f mW" % config_In.batteryUps_Temp_F)
##jwc replace w/ PiUpTimeUps:         print(f" // batteryUps_Temp_F: {config_In.batteryUps_Temp_F:.2f} mW)")
##jwc replace w/ PiUpTimeUps:     except DeviceRangeError as e:
##jwc replace w/ PiUpTimeUps:         print(e)

import piUpTimeUps_2pt0__AlchemyPower

##jwc y import config_Global_File as config_Global_File
##jwc ? import config_Global_File
##jwc o import config_Global_File as cfg
import config_Global_File

##jwc o import io_wrapper as hw
##jwc y import io_Driver_File
##jwc m import io_Driver_Simulator_File as io_Driver_File
import io_Driver_Simulator_File as io_Driver_File

## jwc o import RPi.GPIO as GPIO

##jwc y from gpiozero import Servo

##jwc o from adafruit_crickit import crickit
##jwc y import robohat
##jwc y robohat.init()

import autoPHat_SparkFun_Driver_File

autoPHat_SparkFun_Driver_File.init()
##jwc y autoPHat_SparkFun_Driver_File.runTest()
autoPHat_SparkFun_Driver_File.runTest_Quick()

##jwc n global servo_01_Pan_Degrees 
##jwc n servo_01_Pan_Degrees = 90
autoPHat_SparkFun_Driver_File.servo_Cam_01_Pan_Fn( config_Global_File.servo_01_Pan_Degrees )
autoPHat_SparkFun_Driver_File.servo_Cam_02_Tilt_Fn( config_Global_File.servo_02_Tilt_Degrees )

autoPHat_SparkFun_Driver_File.servo_Arm_03_Fn( config_Global_File.servo_03_Degrees )

##jwc o # make two variables for the motors to make code shorter to type
##jwc o # Right-Side
##jwc o motor_1 = crickit.dc_motor_1
##jwc o # Left-Side
##jwc o motor_2 = crickit.dc_motor_2

##jwc 'crickit' o servo_1 = crickit.servo_1
##jwc 'crickit' o servo_2 = crickit.servo_2

#jwc Due to 'robohat.py' using 'GPIO.setmode(GPIO.BOARD)', then convert from BCM Pin# to Board Pin#
##jwc o myGPIO=24
##jwc o myGPIO_02=25
##jwc o Board#: myGPIO = 18
##jwc o Board#: myGPIO_02 = 22

# jwc: Convert from Board# to BCM#
myGPIO = 24
myGPIO_02 = 25
# gpiozero.exc.GPIOPinInUse: pin 6 is already in use by <gpiozero.PWMOutputDevice object on pin GPIO6, active_high=True, is_active=True>
# gpiozero.exc.GPIOPinInUse: pin 18 is already in use by <gpiozero.PWMOutputDevice object on pin GPIO18, active_high=True, is_active=True>
##jwc bcm# y myGPIO = 6
##jwc bcm#24y myGPIO_02 = 18
##jwc ? myGPIO = 24

##jwc n myGPIO_02 = 25  ## Became hot after 5'
##jwc n myGPIO_02 = 6 ## hot also
## jwc ? myGPIO_02 = 18


##jwc o myCorrection=0.45
#jwc remove correction for smaller range
myCorrection=0.00

maxPW=(2.0+myCorrection)/1000
minPW=(1.0-myCorrection)/1000

##jwc y servo = Servo(myGPIO)
##jwc y servo_02 = Servo(myGPIO_02)

##jwc y servo = Servo(myGPIO,min_pulse_width=minPW,max_pulse_width=maxPW)
##jwc y servo_02 = Servo(myGPIO_02,min_pulse_width=minPW,max_pulse_width=maxPW)

##jwc n servo.mid()
##jwc n servo_02.mid()

##jwc 'crickit' ##jwc o GPIO.setmode(GPIO.BCM)
##jwc 'crickit' ##jwc o GPIO.setwarnings(False)
##jwc 'crickit' 
##jwc 'crickit' 
##jwc 'crickit' ##jwc o servo = 22, 22 is Board Pin #
##jwc 'crickit' ##jwc y servo = 22  # jwc bcm=25
##jwc 'crickit' ##jwc o servo_Pin = 18  # jwc bcm=24
##jwc 'crickit' ##jwc o servo_02_Pin = 22  # jwc bcm=25
##jwc 'crickit' 
##jwc 'crickit' ##jwc o bug: GPIO.setmode(GPIO.BOARD).py
##jwc 'crickit' ##jwc o GPIO.setmode(GPIO.BOARD)
##jwc 'crickit' ##TODO jwc jittery: GPIO.setup(servo_Pin, GPIO.OUT)
##jwc 'crickit' ##jwc o GPIO.setup(servo_02_Pin, GPIO.OUT)
##jwc 'crickit' 
##jwc 'crickit' # jwc: 5ms = 0.005s -> 1 / 200 = 200Hz
##jwc 'crickit' ##TODO jwc jittery: servo = GPIO.PWM(servo_Pin, 200)   # frequency is 500Hz, so each pulse is 5ms wide
##jwc 'crickit' ##jwc o servo_02 = GPIO.PWM(servo_02_Pin, 200)   # frequency is 500Hz, so each pulse is 5ms wide
##jwc 'crickit' 
##jwc 'crickit' # servos will be fully left at 0.5ms, centred at 1.5ms and fully servoPwm_PositionMin at 2.5ms
##jwc 'crickit' #
##jwc 'crickit' servoPwm_PositionMax = 50/5
##jwc 'crickit' servoPwm_PositionMid = 150/5
##jwc 'crickit' servoPwm_PositionMin = 250/5
##jwc 'crickit' 
##jwc 'crickit' ##TODO jwc jitter: servo.start(servoPwm_PositionMid) # start it at 50% - should be servoPwm_PositionMid of servo
##jwc 'crickit' servo_02.start(servoPwm_PositionMid) # start it at 50% - should be servoPwm_PositionMid of servo
##jwc 'crickit' #p.ChangeDutyCycle(100)

##jwc o AiCam Gen 1
##jwc o 
##jwc o  Raspberry Pi camera module (requires picamera package)
##jwc o  from camera_pi import Camera_File
##jwc o 
##jwc o def gen(camera):
##jwc o     """Video streaming generator function."""
##jwc o     while True:
##jwc o         frame = camera.get_frame()
##jwc o         yield (b'--frame\r\n'
##jwc o                b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
##jwc o          
##jwc o @app_Cl_Ob.route('/video_feed')
##jwc o def video_feed():
##jwc o     """Video streaming route. Put this in the src attribute of an img tag."""
##jwc o     return Response(gen(Camera()), mimetype='multipart/x-mixed-replace; boundary=frame')


# AiCam Gen 2.0
#

# import camera driver
##jwc o if os.environ.get('CAMERA'):
##jwc o     camera_Cl = import_module('camera_' + os.environ['CAMERA']).camera_Cl
##jwc o     print("*** DEBUG: Camera-02a: camera_" + os.environ['CAMERA'])
##jwc o else:
##jwc o     ##jwc o from camera import Camera_File
##jwc o     # Default to most sophisticated tech
##jwc o     from camera_OpenCv_File import camera_Cl
##jwc o     print("*** DEBUG: Camera-02b: camera_opencv")

##jwc o from camera import Camera_File
# Default to most sophisticated tech
from camera_OpenCv_File import camera_Cl
print("*** DEBUG: Camera: camera_opencv")


# jwc 2020-1223 StreamVideoToWebBrowser-AdrianRosebrock
#
##jwc o from pyimagesearch.motion_detection import singleMotionDetector_Cl
from motion_detection.singleMotionDetector_File import singleMotionDetector_Cl
from imutils.video import VideoStream
import threading
import argparse
import datetime
import imutils
import time
import cv2

from collections import defaultdict

##jwc 2.1 
##
# define names of each possible ArUco tag OpenCV supports
ARUCO_DICT = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
    "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
    "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
    "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
    "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}

##jwc 2.0
##

# initialize the output frame and a lock used to ensure thread-safe
# exchanges of the output frames (useful for multiple browsers/tabs
# are viewing tthe stream)
outputFrame = None
lock = threading.Lock()

score_Targeted_Dict = defaultdict(int)

##jwc o # initialize a flask object
##jwc o app_Cl_Ob = Flask(__name__)

# initialize the video stream and allow the camera sensor to
# warmup
#videoStream_Cl_Ob = VideoStream(usePiCamera=1).start()
print("[INFO] starting video stream...")
videoStream_Cl_Ob = VideoStream(src=0).start()
time.sleep(2.0)


def detect_Motions_Fn(frameCount):
    # grab global references to the video stream, output frame, and
    # lock variables
    global videoStream_Cl_Ob, outputFrame, lock

    # initialize the motion detector and the total number of frames
    # read thus far
    motionDetect_Cl_Ob = singleMotionDetector_Cl(accumWeight=0.1)
    total = 0

    # loop over frames from the video stream
    while True:
        # read the next frame from the video stream, resize it,
        # convert the frame to grayscale, and blur it
        frame = videoStream_Cl_Ob.read()
        frame = imutils.resize(frame, width=400)
        # jwc rotate 180-degrees to flip image, since cam is wrongly upside-down
        ##jwc not work as time stamp upside down:  frame = imutils.rotate(frame, 180)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (7, 7), 0)
        
        # jwc Frame is originally right-side up, yet timestamp-print is upside down
        #     So, flip upside down before timestamp-print, then re-flip after
        frame = imutils.rotate(frame, 180)

        # grab the current timestamp and draw it on the frame
        timestamp = datetime.datetime.now()
        # cv2.putText(frame, timestamp.strftime(
        # 	"%A %d %B %Y %I:%M:%S%p"), (10, frame.shape[0] - 10),
        # 	cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 255), 1)
        cv2.putText(frame, timestamp.strftime(
            "%A %d %B %Y %I:%M:%S%p"), (10, frame.shape[0] - 10),
            cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 255), 1)
        # jwc Frame is originally right-side up, yet timestamp-print is upside down
        #     So, flip upside down before timestamp-print, then re-flip after
        frame = imutils.rotate(frame, 180)

        # if the total number of frames has reached a sufficient
        # number to construct a reasonable background model, then
        # continue to process the frame
        if total > frameCount:
            # detect motion in the image
            motion = motionDetect_Cl_Ob.detect(gray)

            # cehck to see if motion was found in the frame
            if motion is not None:
                # unpack the tuple and draw the box surrounding the
                # "motion area" on the output frame
                (thresh, (minX, minY, maxX, maxY)) = motion
                cv2.rectangle(frame, (minX, minY), (maxX, maxY),
                    (0, 0, 255), 2)
        
        # update the background model and increment the total number
        # of frames read thus far
        motionDetect_Cl_Ob.update(gray)
        total += 1

        # acquire the lock, set the output frame, and release the
        # lock
        with lock:
            outputFrame = frame.copy()

## jwc 2.1
##

# Generate 'outputFrame' for later client request
#        
##jwc o def detect_Motions_Fn(frameCount):
def detect_Motions_And_ArucoMarkers_Fn(frameCount):
    # grab global references to the video stream, output frame, and
    # lock variables
    global videoStream_Cl_Ob, outputFrame, lock

    # initialize the motion detector and the total number of frames
    # read thus far
    motionDetect_Cl_Ob = singleMotionDetector_Cl(accumWeight=0.1)
    total = 0

    # loop over frames from the video stream
    while True:
        # read the next frame from the video stream, resize it,
        # convert the frame to grayscale, and blur it
        frame = videoStream_Cl_Ob.read()
        ##jwc o frame = imutils.resize(frame, width=400)
        frame = imutils.resize(frame, width=1000)

        # jwc Frame is originally right-side up, yet timestamp-print is upside down
        #     So, flip upside down before timestamp-print, then re-flip after
        frame = imutils.rotate(frame, 180)

        # detect ArUco markers in the input frame
        (corners, ids, rejected) = cv2.aruco.detectMarkers(frame, arucoDict, parameters=arucoParams)

        # verify *at least* one ArUco marker was detected
        if len(corners) > 0:
            # flatten the ArUco IDs list
            ids = ids.flatten()
    
            # loop over the detected ArUCo corners
            for (markerCorner, markerID) in zip(corners, ids):
                # extract the marker corners (which are always returned
                # in top-left, top-right, bottom-right, and bottom-left
                # order)
                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners
    
                # convert each of the (x, y)-coordinate pairs to integers
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))
    
                # draw the bounding box of the ArUCo detection
                cv2.line(frame, topLeft, topRight, (0, 255, 0), 2)
                cv2.line(frame, topRight, bottomRight, (0, 255, 0), 2)
                cv2.line(frame, bottomRight, bottomLeft, (0, 255, 0), 2)
                cv2.line(frame, bottomLeft, topLeft, (0, 255, 0), 2)
    
                # compute and draw the center (x, y)-coordinates of the
                # ArUco marker
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                cv2.circle(frame, (cX, cY), 4, (0, 0, 255), -1)
    
                # draw the ArUco marker ID on the frame
                # * Maker ID: Red Color
                ##jwc o cv2.putText(frame, str(markerID), (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                # https://www.geeksforgeeks.org/python-opencv-cv2-puttext-method/
                # * LineTypes: Recommended: LINE_AA = 8-connected line
                cv2.putText(frame, str(markerID), (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2, cv2.LINE_AA)
                score_Targeted_Dict[str(markerID)] += 1
                print("*** DEBUG: /detect_Motions_And_ArucoMarkers_Fn: markerID=" + str(markerID) + " markerID%10=" + str(markerID % 10))
                print("*** *** DEBUG: /detect_Motions_And_ArucoMarkers_Fn: score_Targeted_Dict= " + str(score_Targeted_Dict))

        ##jwc o 2.1 # show the output frame
        ##jwc o 2.1 cv2.imshow("Frame", frame)


        # jwc rotate 180-degrees to flip image, since cam is wrongly upside-down
        ##jwc not work as time stamp upside down:  frame = imutils.rotate(frame, 180)        # https://www.geeksforgeeks.org/python-opencv-cv2-puttext-method/
        # * LineTypes: Recommended: LINE_AA = 8-connected line
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (7, 7), 0)
        
        # jwc Frame is originally right-side up, yet timestamp-print is upside down
        #     So, flip upside down before timestamp-print, then re-flip after
        ##jwc m frame = imutils.rotate(frame, 180)
        
        # grab the current timestamp and draw it on the frame
        timestamp = datetime.datetime.now()
        # cv2.putText(frame, timestamp.strftime(
        # 	"%A %d %B %Y %I:%M:%S%p"), (10, frame.shape[0] - 10),
        # 	cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 255), 1) 
        # * Date Stamp: Green Color
        # * https://www.geeksforgeeks.org/python-opencv-cv2-puttext-method/
        # * LineTypes: Recommended: LINE_AA = 8-connected line
        ##jwc o cv2.putText(frame, timestamp.strftime("%A %d %B %Y %I:%M:%S%p"), (10, frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 255), 1)
        cv2.putText(frame, timestamp.strftime("%A %d %B %Y %I:%M:%S%p"), (10, frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2, cv2.LINE_AA)

        ##jwc m # jwc Frame is originally right-side up, yet timestamp-print is upside down
        ##jwc m #     So, flip upside down before timestamp-print, then re-flip after
        ##jwc m frame = imutils.rotate(frame, 180)

        # if the total number of frames has reached a sufficient
        # number to construct a reasonable background model, then
        # continue to process the frame
        if total > frameCount:
            # detect motion in the image
            motion = motionDetect_Cl_Ob.detect(gray)

            # check to see if motion was found in the frame
            if motion is not None:
                # unpack the tuple and draw the box surrounding the
                # "motion area" on the output frame
                (thresh, (minX, minY, maxX, maxY)) = motion
                cv2.rectangle(frame, (minX, minY), (maxX, maxY), (0, 0, 255), 2)
        
        # update the background model and increment the total number
        # of frames read thus far
        motionDetect_Cl_Ob.update(gray)
        total += 1

        # jwc Frame is originally right-side up, yet timestamp-print is upside down
        #     So, flip upside down before timestamp-print, then re-flip after
        frame = imutils.rotate(frame, 180)

        # acquire the lock, set the output frame, and release the
        # lock
        with lock:
            outputFrame = frame.copy()

# Return generated 'outputFrame' for current client request
def generate():
    # grab global references to the output frame and lock variables
    global outputFrame, lock

    # loop over frames from the output stream
    while True:
        # wait until the lock is acquired
        with lock:
            # check if the output frame is available, otherwise skip
            # the iteration of the loop
            if outputFrame is None:
                continue

            # encode the frame in JPEG format: 'im(age) encode' = 'imencode'
            (flag, encodedImage) = cv2.imencode(".jpg", outputFrame)

            # ensure the frame was successfully encoded
            if not flag:
                continue

        # yield the output frame in the byte format
        yield(b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + 
            bytearray(encodedImage) + b'\r\n')


import logging
log = logging.getLogger('werkzeug')
##jwc o log.setLevel(logging.ERROR)
log.setLevel(logging.INFO)


# jwc ~/01-Jwc/2020-1205-0640-RpiRover1-DayDevelops/RpiRover1-master/
# jwc 'sys.path.append('db') does work, depsite 'lint' not understand for 'import'
sys.path.append('db')
import robotProperties_Db_Cl_File

DB = robotProperties_Db_Cl_File.robotProperties_Db_Cl()

dictionary_ReturnValues = DB.read_LeftRight_MotorTrim_Fn()
print("*** DEBUG: DB.read_LeftRight_MotorTrim_Fn: " + json.dumps( dictionary_ReturnValues ))
config_Global_File.left_motor_trim = dictionary_ReturnValues['L']
config_Global_File.right_motor_trim = dictionary_ReturnValues['R']
print ("*** DEBUG: DB: config_Global_File.left_motor_trim: " + str( config_Global_File.left_motor_trim ) + ", config_Global_File.right_motor_trim: " + str( config_Global_File.right_motor_trim ))

dictionary_ReturnValues = DB.read_Heartbeat_Freq_Fn()
print("*** DEBUG: DB.read_Heartbeat_Freq_Fn: " + json.dumps( dictionary_ReturnValues ))
config_Global_File.heartbeat_freq = dictionary_ReturnValues['H']
print ("*** DEBUG: DB: config_Global_File.heartbeat_freq: " + str( config_Global_File.heartbeat_freq ))


##jwc yo app_Cl_Ob = Flask(__name__)

app_Cl_Ob = Flask(__name__, static_url_path='/static')

    
@app_Cl_Ob.route('/')
def index():
    """Video streaming home page."""
    return render_template('index.html')


# jwc 2020-1223 AiCam 2.0 StreamVideoToWebBrowser-AdrianRosebrock
#
@app_Cl_Ob.route("/video_feed")
def video_feed():
    # return the response generated along with the specific media
    # type (mime type)
    return Response(generate(),	mimetype = "multipart/x-mixed-replace; boundary=frame")


# Immobilizes sytem (chocks on) after 'timeout' seconds 
def watchdog_timer():
    while config_Global_File.watchdog_Alive_Bool:
        # jwc: Pauses every 1sec
        ##jwc o time.sleep(1)
        ##jwc
        ##jwc y  decrease for more real-time stats: time.sleep(5)
        ##jwc still too long: time.sleep(1)
        ##jwc time.sleep(.10)
        ##jwc y  realize this not affect HUD Stats, so keep as is to prevent premature disconnect
        time.sleep(5)
        
        if config_Global_File.watchdog_Start_On_Bool:
            config_Global_File.watchdog_Cycles_Now += 1
            ##jwc print("*** DEBUG: config_Global_File.watchdog_Cycles_Now: " + str(config_Global_File.watchdog_Cycles_Now))
            # jwc: appears that beyond 10sec is bad disconnect, 
            #      so engage 'chocks/disable bot', if not already
            if config_Global_File.watchdog_Cycles_Now > config_Global_File.timeout_Cycles_MAX and not config_Global_File.chocks:
                chocks_on()
            if config_Global_File.watchdog_Cycles_Now <= config_Global_File.timeout_Cycles_MAX and config_Global_File.chocks:
                chocks_off()

# Handler for a clean shutdown when pressing Ctrl-C
def signal_handler(signal, frame):
    io_Driver_File.light_blue_blink(0.1)
    config_Global_File.watchdog_Alive_Bool = False
    config_Global_File.camera_active = False
    brakes_on()
    # jwc: Wait until thread terminates
    watchDog.join()
    ##jwc o http_server.close()
    io_Driver_File.light_blue_off()
    sys.exit(0)

# Handler for explorer-hat touchpads
def touch_handler(channel, event):

    if channel == 1:
        #jwc o 
        # config_Global_File.blue = not config_Global_File.blue
        # if config_Global_File.blue:
        #     io_Driver_File.light_blue_on()
        #     io_Driver_File.output_one_on()
        # else:
        #     io_Driver_File.light_blue_off()
        #     io_Driver_File.output_one_off()

        ##jwc n AttributeError: servo_01_Pan = autoPHat_SparkFun_Driver_File.servo_Cam_01_Pan_PositiionGet_Fn()
        ##jwc n config_Global_File.servo_01_Pan_Degrees = autoPHat_SparkFun_Driver_File.servo_Cam_01_Pan_PositionGet_Fn()

        config_Global_File.servo_01_Pan_Degrees = config_Global_File.servo_01_Pan_Degrees - 10
        if config_Global_File.servo_01_Pan_Degrees < 0: 
            config_Global_File.servo_01_Pan_Degrees = 0
        autoPHat_SparkFun_Driver_File.servo_Cam_01_Pan_Fn( config_Global_File.servo_01_Pan_Degrees )
        print("*** DEBUG: S1: servo: pan: " + str(config_Global_File.servo_01_Pan_Degrees))
        
    if channel == 2:
        #jwc o 
        # config_Global_File.yellow = not config_Global_File.yellow
        # if config_Global_File.yellow:
        #     io_Driver_File.light_yellow_on()
        #     io_Driver_File.output_two_on()
        # else:
        #     io_Driver_File.light_yellow_off()
        #     io_Driver_File.output_two_off()

        ##jwc n AttributeError: servo_01_Pan = autoPHat_SparkFun_Driver_File.servo_Cam_01_Pan_PositiionGet_Fn()
        ##jwc n config_Global_File.servo_01_Pan_Degrees = autoPHat_SparkFun_Driver_File.servo_Cam_01_Pan_PositionGet_Fn()

        config_Global_File.servo_01_Pan_Degrees = config_Global_File.servo_01_Pan_Degrees + 10
        if config_Global_File.servo_01_Pan_Degrees > 180: 
            config_Global_File.servo_01_Pan_Degrees = 180
        autoPHat_SparkFun_Driver_File.servo_Cam_01_Pan_Fn( config_Global_File.servo_01_Pan_Degrees )
        print("*** DEBUG: S1: servo: pan: " + str(config_Global_File.servo_01_Pan_Degrees))

    if channel == 3:
        config_Global_File.chocks = not config_Global_File.chocks
        # jwc: Chocks set to True: Admin Lock
        if config_Global_File.chocks:
            # jwc: Since Motors not free to operate, Watchdog not needed
            config_Global_File.watchdog_Start_On_Bool = False
            chocks_on()
        # jwc: Chocks set to False: Admin Unlock
        else:
            # jwc: Since Motors are free to operate, Watchdog is needed
            config_Global_File.watchdog_Start_On_Bool = True
            chocks_off()

    if channel == 4:
        io_Driver_File.light_green_blink(0.1)
        #jwc o 
        # config_Global_File.green = True
        # ##jwc o time.sleep(5)
        # if config_Global_File.chocks:
        #     io_Driver_File.light_green_on()
        #     ##jwc o os.system("sudo -s shutdown -h now")
        # else:
        #     io_Driver_File.light_green_off()
        #     config_Global_File.green = False

        ##jwc n config_Global_File.temp = autoPHat_SparkFun_Driver_File.servo_Cam_02_Tilt_PositionGet_Fn()

        ##jwc n
        # servoCam02Tilt = servoCam02Tilt + 10
        # autoPHat_SparkFun_Driver_File.servo_Cam_02_Tilt_Fn( servoCam02Tilt ) 
        # print("*** DEBUG: S2: servo: tilt: " + str(autoPHat_SparkFun_Driver_File.servo_Cam_02_Tilt_PositionGet_Fn()))

        config_Global_File.servo_02_Tilt_Degrees = config_Global_File.servo_02_Tilt_Degrees - 10
        if config_Global_File.servo_02_Tilt_Degrees < 0: 
            config_Global_File.servo_02_Tilt_Degrees = 0
        autoPHat_SparkFun_Driver_File.servo_Cam_02_Tilt_Fn( config_Global_File.servo_02_Tilt_Degrees )
        print("*** DEBUG: S2: servo: tilt: " + str(config_Global_File.servo_02_Tilt_Degrees))

    if channel == 5:
        io_Driver_File.light_green_blink(0.1)
        #jwc o 
        # config_Global_File.green = True
        # ##jwc o time.sleep(5)
        # if config_Global_File.chocks:
        #     io_Driver_File.light_green_on()
        #     ##jwc o os.system("sudo -s shutdown -h now")
        # else:
        #     io_Driver_File.light_green_off()
        #     config_Global_File.green = False

        ##jwc n config_Global_File.temp = autoPHat_SparkFun_Driver_File.servo_Cam_02_Tilt_PositionGet_Fn()

        ##jwc n
        # servoCam02Tilt = servoCam02Tilt + 10
        # autoPHat_SparkFun_Driver_File.servo_Cam_02_Tilt_Fn( servoCam02Tilt ) 
        # print("*** DEBUG: S2: servo: tilt: " + str(autoPHat_SparkFun_Driver_File.servo_Cam_02_Tilt_PositionGet_Fn()))

        config_Global_File.servo_02_Tilt_Degrees = config_Global_File.servo_02_Tilt_Degrees + 10
        if config_Global_File.servo_02_Tilt_Degrees > 180: 
            config_Global_File.servo_02_Tilt_Degrees = 180
        autoPHat_SparkFun_Driver_File.servo_Cam_02_Tilt_Fn( config_Global_File.servo_02_Tilt_Degrees )
        print("*** DEBUG: S2: servo: tilt: " + str(config_Global_File.servo_02_Tilt_Degrees))

def brakes_on():
    config_Global_File.brakes = True
    config_Global_File.left_motor = 0
    config_Global_File.right_motor = 0
    ##jwc o io_Driver_File.motor_one_speed(config_Global_File.right_motor)
    ##jwc o io_Driver_File.motor_two_speed(config_Global_File.left_motor)
    
# jwc: Motors free to operate: Lo-Level: User-Level
def brakes_off():
    config_Global_File.brakes = False
    config_Global_File.watchdog_Cycles_Now = 0
    
def chocks_on():
    config_Global_File.chocks = True
    brakes_on()
    io_Driver_File.light_red_blink(0.2)

# jwc: Motors free to operate: Hi-Level: Admin-Level ~ Overrides User-Level for Security/Safety
def chocks_off():
    config_Global_File.chocks = False
    brakes_off()
    io_Driver_File.light_red_off()


""" jwc y
# URL for motor control - format: /motor?l=[speed]&r=[speed]
@app_Cl_Ob.route('/motor')
def motor():
    left = request.args.get('l')
    ##o if left and not config_Global_File.chocks:
    if left:
        left = int(left)
        if left >= -100 and left <= 100:
            ##o config_Global_File.left_motor = left
            ##o io_Driver_File.motor_two_speed(config_Global_File.left_motor)
            left_normalized = (left / 100 )
            motor_1.throttle = left_normalized
            time.sleep(3)
            motor_1.throttle = -1 * left_normalized
            time.sleep(3)
            motor_1.throttle = 0
            time.sleep(3)
  
            servo_1.angle = 90
            time.sleep(3)
            servo_1.angle = 135
            time.sleep(3)
            servo_1.angle = 45
            time.sleep(3)

            print("motor-left: " + str(servoPwm_PositionMax) + " " + str(left_normalized))
    return 'ok'
 """

# URL for motor control - format: /motor?l=[speed]&r=[speed]
@app_Cl_Ob.route('/motor')
def motor():
    left = request.args.get('l')
    right = request.args.get('r')

    print("*** *** DEBUG: left: " + str(left) + " right: " + str(right))

    left_normalized = 0
    right_normalized = 0

    if left and not config_Global_File.chocks:
        left_Int = int(left)
        config_Global_File.left_motor = left_Int
        left_Absolute = abs( left_Int )
        if left_Int >= -100 and left_Int <= 100:
            ##jwc yo config_Global_File.left_motor = left
            ##jwc o io_Driver_File.motor_two_speed(config_Global_File.left_motor)
            ##jwc o left_normalized = (left / 100 )
            if left_Int >= 0:
                ##jwc y autoPHat_SparkFun_Driver_File.motorLeft_Fn( left_Int )
                autoPHat_SparkFun_Driver_File.motorLeft_Fn( (left_Int/100) * 250 )
                ##jwc n autoPHat_SparkFun_Driver_File.motorRight_Fn( (left_Int/100) * 250 )

                print("*** DEBUG: L1: motor: L " + str((left_Int/100) * 250))
            elif left_Int < 0:
                ##jwc y autoPHat_SparkFun_Driver_File.motorLeft_Fn( left_Int )
                autoPHat_SparkFun_Driver_File.motorLeft_Fn( (left_Int/100) * 250 )
                ##jwc n autoPHat_SparkFun_Driver_File.motorRight_Fn( (left_Int/100) * 250 )

                print("*** DEBUG: L2: motor: L " + str((left_Int/100) * 250))
            else:
                print("*** Error: Invalid Value: left_Int: ", left_Int)
            ##jwc o motor_1.throttle = left_normalized

    if right and not config_Global_File.chocks:
        right_Int = int(right)
        config_Global_File.right_motor = right_Int
        right_Absolute = abs( right_Int )
        if right_Int >= -100 and right_Int <= 100:
            ##jwc o config_Global_File.right_motor = right
            ##jwc o io_Driver_File.motor_one_speed(config_Global_File.right_motor)
            ##jwc o right_normalized = (right / 100 )
            if right_Int >= 0:
                ##jwc y autoPHat_SparkFun_Driver_File.motorRight_Fn( right_Int )
                autoPHat_SparkFun_Driver_File.motorRight_Fn( (right_Int/100) * 250 )
                ##jwc n autoPHat_SparkFun_Driver_File.motorRight_Fn( -1 * (right_Int/100) * 250 )

                print("*** DEBUG: R1: motor: R " + str((right_Int/100) * 250))
            elif right_Int < 0:
                ##jwc y autoPHat_SparkFun_Driver_File.motorRight_Fn( right_Int )
                autoPHat_SparkFun_Driver_File.motorRight_Fn( (right_Int/100) * 250 )
                ##jwc n autoPHat_SparkFun_Driver_File.motorRight_Fn( -1 * (right_Int/100) * 250 )

                print("*** DEBUG: R2: motor: R " + str((right_Int/100) * 250))
            else:
                print("*** Error: Invalid Value: right_Int: ", right_Int)
            ##jwc o motor_2.throttle = right_normalized

    ##jwc y print("*** DEBUG: motor: l" + str(left_normalized) + " r" + str(right_normalized))
    ##jwc y print("*** DEBUG: motor: l" + str(left_Int) + " r" + str(right_Int)
    ##jwc yn print("*** DEBUG: motor: l" + str((left_Int/100) * 250) + " r" + str((right_Int/100) * 250))
    return 'ok'

# URL for motor control - format: /motor?l=[speed]&r=[speed]
@app_Cl_Ob.route('/motor_for_turn')
def motor_for_turn():
    left = request.args.get('l')
    right = request.args.get('r')

    print("*** *** DEBUG: left: " + str(left) + " right: " + str(right))

    left_normalized = 0
    right_normalized = 0

    if left and not config_Global_File.chocks:
        left_Int = int(left)
        config_Global_File.left_motor = left_Int
        left_Absolute = abs( left_Int )
        if left_Int >= -100 and left_Int <= 100:
            ##jwc yo config_Global_File.left_motor = left
            ##jwc o io_Driver_File.motor_two_speed(config_Global_File.left_motor)
            ##jwc o left_normalized = (left / 100 )
            if left_Int >= 0:
                ##jwc y autoPHat_SparkFun_Driver_File.motorLeft_Fn( left_Int )
                autoPHat_SparkFun_Driver_File.motorLeft_Fn( (left_Int/100) * 250 )
                ##jwc n autoPHat_SparkFun_Driver_File.motorRight_Fn( (left_Int/100) * 250 )

                print("*** DEBUG: L1: motor: L " + str((left_Int/100) * 250))
            elif left_Int < 0:
                ##jwc y autoPHat_SparkFun_Driver_File.motorLeft_Fn( left_Int )
                autoPHat_SparkFun_Driver_File.motorLeft_Fn( (left_Int/100) * 250 )
                ##jwc n autoPHat_SparkFun_Driver_File.motorRight_Fn( (left_Int/100) * 250 )

                print("*** DEBUG: L2: motor: L " + str((left_Int/100) * 250))
            else:
                print("*** Error: Invalid Value: left_Int: ", left_Int)
            ##jwc o motor_1.throttle = left_normalized

    if right and not config_Global_File.chocks:
        right_Int = int(right)
        # Since turning, need to invert sign of 'right'
        right_Int = -1 * right_Int
        config_Global_File.right_motor = right_Int
        right_Absolute = abs( right_Int )
        if right_Int >= -100 and right_Int <= 100:
            ##jwc o config_Global_File.right_motor = right
            ##jwc o io_Driver_File.motor_one_speed(config_Global_File.right_motor)
            ##jwc o right_normalized = (right / 100 )
            if right_Int >= 0:
                ##jwc y autoPHat_SparkFun_Driver_File.motorRight_Fn( right_Int )
                autoPHat_SparkFun_Driver_File.motorRight_Fn( (right_Int/100) * 250 )
                ##jwc n autoPHat_SparkFun_Driver_File.motorRight_Fn( -1 * (right_Int/100) * 250 )

                print("*** DEBUG: R1: motor: R " + str((right_Int/100) * 250))
            elif right_Int < 0:
                ##jwc y autoPHat_SparkFun_Driver_File.motorRight_Fn( right_Int )
                autoPHat_SparkFun_Driver_File.motorRight_Fn( (right_Int/100) * 250 )
                ##jwc n autoPHat_SparkFun_Driver_File.motorRight_Fn( -1 * (right_Int/100) * 250 )

                print("*** DEBUG: R2: motor: R " + str((right_Int/100) * 250))
            else:
                print("*** Error: Invalid Value: right_Int: ", right_Int)
            ##jwc o motor_2.throttle = right_normalized

    ##jwc y print("*** DEBUG: motor: l" + str(left_normalized) + " r" + str(right_normalized))
    ##jwc y print("*** DEBUG: motor: l" + str(left_Int) + " r" + str(right_Int)
    ##jwc yn print("*** DEBUG: motor: l" + str((left_Int/100) * 250) + " r" + str((right_Int/100) * 250))
    return 'ok'


@app_Cl_Ob.route('/servo_Cam_01_Pan_Degrees_FrontEnd_Fn')
def servo_Cam_01_Pan_Degrees_FrontEnd_Fn():
    servoDegreesInt = int(request.args.get('servo_Cam_01_Pan_Degrees_FrontEnd_Id'))

    if servoDegreesInt < 0: 
        servoDegreesInt = 0
    elif servoDegreesInt > 180: 
        servoDegreesInt = 180
    config_Global_File.servo_01_Pan_Degrees = servoDegreesInt
    autoPHat_SparkFun_Driver_File.servo_Cam_01_Pan_Fn( config_Global_File.servo_01_Pan_Degrees )    
    print("*** DEBUG: /servo_Cam_01_Pan_Degrees_FrontEnd_Fn: " + str(servoDegreesInt))
    return 'ok'


@app_Cl_Ob.route('/servo_Cam_02_Tilt_Degrees_FrontEnd_Fn')
def servo_Cam_02_Tilt_Degrees_FrontEnd_Fn():
    servoDegreesInt = int(request.args.get('servo_Cam_02_Tilt_Degrees_FrontEnd_Id'))

    if servoDegreesInt < 0: 
        servoDegreesInt = 0
    elif servoDegreesInt > 180: 
        servoDegreesInt = 180
    config_Global_File.servo_02_Tilt_Degrees = servoDegreesInt
    autoPHat_SparkFun_Driver_File.servo_Cam_02_Tilt_Fn( config_Global_File.servo_02_Tilt_Degrees )
    print("*** DEBUG: /servo_Cam_02_Tilt_Degrees_FrontEnd_Fn: " + str(servoDegreesInt))
    return 'ok'

@app_Cl_Ob.route('/servo_Arm_03_Degrees_FrontEnd_Fn')
def servo_Arm_03_Degrees_FrontEnd_Fn():
    servoDegreesInt = int(request.args.get('servo_Arm_03_Degrees_FrontEnd_Id'))

    if servoDegreesInt < 0: 
        servoDegreesInt = 0
    elif servoDegreesInt > 180: 
        servoDegreesInt = 180
    config_Global_File.servo_03_Degrees = servoDegreesInt
    autoPHat_SparkFun_Driver_File.servo_Arm_03_Fn( config_Global_File.servo_03_Degrees )
    print("*** DEBUG: /servo_Arm_03_Degrees_FrontEnd_Fn: " + str(servoDegreesInt))
    return 'ok'


# URL for motor control - format: /motor?l=[speed]&r=[speed]
@app_Cl_Ob.route('/motorTrim')
def motorTrim():
    left = request.args.get('l')
    right = request.args.get('r')

    print("*** *** DEBUG: motorTrim() Pre : left: " + str(left) + " right: " + str(right))

    config_Global_File.left_motor_trim += int( left )
    config_Global_File.right_motor_trim += int( right )

    DB.write_LeftRight_MotorTrim_Fn(config_Global_File.left_motor_trim, config_Global_File.right_motor_trim)

    print("*** *** DEBUG: motorTrim() Post: left: " + str(config_Global_File.left_motor_trim) + " right: " + str(config_Global_File.right_motor_trim))

    return 'ok'

# URL for motor control - format: /motor?l=[speed]&r=[speed]
@app_Cl_Ob.route('/heartbeat_Freq_Mod_IncDec_Fn')
def heartbeat_Freq_IncDec_Fn():
    incdec = request.args.get('incdec')

    print("*** *** DEBUG: heartbeat_Freq_Mod_IncDec_Fn(): incdec: " + str(incdec))

    if (config_Global_File.heartbeat_freq + int( incdec )) < 0:
        print("*** *** DEBUG: heartbeat_Freq_Mod_IncDec_Fn(): config_Global_File.heartbeat_freq < 0: Invalid")
    else:
        config_Global_File.heartbeat_freq += int( incdec )
        DB.write_Heartbeat_Freq_Fn(config_Global_File.heartbeat_freq)
        print("*** *** DEBUG: heartbeat_Freq_Mod_IncDec_Fn(): config_Global_File.heartbeat_freq >= 0: Valid"  )
    return 'ok'

""" jwc o
 # URL for joystick input - format: /joystick?x=[x-axis]&y=[y-axis]
@app_Cl_Ob.route('/joystick')
def joystick():
    config_Global_File.watchdog_Cycles_Now = 0
    x_axis = int(request.args.get('x'))
    y_axis = int(request.args.get('y'))
    x_axis = -1 * max( min(x_axis, 100), -100)
    y_axis = max( min(y_axis, 100), -100)
    v = (100-abs(x_axis)) * (y_axis/100) + y_axis
    w = (100-abs(y_axis)) * (x_axis/100) + x_axis
    r = int((v+w) / 2)
    l = int((v-w) / 2)
    if not config_Global_File.chocks:
        config_Global_File.right_motor = r
        config_Global_File.left_motor = l
        io_Driver_File.motor_one_speed(config_Global_File.right_motor)
        io_Driver_File.motor_two_speed(config_Global_File.left_motor)
    return 'ok'
 """

# URL to remote control touchpads 1-4 on explorer-hat
@app_Cl_Ob.route('/touchpad')
def touchpad():
    pad = request.args.get('pad')
    if pad:
        touch_handler(int(pad), True)
    return 'ok'

# URL for heartbeat requests (resets watchdog timer)    
# Returns JSON object with status data
@app_Cl_Ob.route('/heartbeat')
def heartbeat():
    config_Global_File.watchdog_Cycles_Now = 0
    output = {}
    output['b'] = config_Global_File.blue
    output['y'] = config_Global_File.yellow
    output['c'] = config_Global_File.chocks
    output['g'] = config_Global_File.green
    output['f'] = config_Global_File.video_fps
    output['v'] = config_Global_File.video_status
    
    output['l'] = config_Global_File.left_motor
    ##jwc o output['l'] = motor_1.throttle
    output['r'] = config_Global_File.right_motor
    ##jwc o output['r'] = motor_2.throttle

    # jwc 
    # 
    output['lt'] = config_Global_File.left_motor_trim
    output['rt'] = config_Global_File.right_motor_trim

    output['hf'] = config_Global_File.heartbeat_freq

    output['s1'] = config_Global_File.servo_01_Pan_Degrees
    output['s2'] = config_Global_File.servo_02_Tilt_Degrees
    output['s3'] = config_Global_File.servo_03_Degrees
    output['s4'] = config_Global_File.servo_04_Degrees

    output['i1'] = io_Driver_File.input_one_read()
    output['i2'] = io_Driver_File.input_two_read()
    output['i3'] = io_Driver_File.input_three_read()
    output['i4'] = io_Driver_File.input_four_read()
    ##jwc o output['a1'] = io_Driver_File.analog_one_read()
    ##jwc o output['a2'] = io_Driver_File.analog_two_read()
    ##jwc o output['a3'] = io_Driver_File.analog_three_read()
    ##jwc o output['a4'] = io_Driver_File.analog_four_read()

    output['sc'] = str( score_Targeted_Dict )

    ## jwc replace w/ PiUpTimeUps: batteryUps_Read_Fn( config_Global_File )
    ##jwc n  get_VoltageAndTemp_Status_Fn( config_Global_File )
    piUpTimeUps_2pt0__AlchemyPower.get_VoltageAndTemp_Status_Fn( config_Global_File )
    output['bvi'] = f'{config_Global_File.batteryUps_Volts_Input_V:.2f}'
    output['bvo'] = f'{config_Global_File.batteryUps_Volts_Output_V:.2f}'
    output['bvb'] = f'{config_Global_File.batteryUps_Volts_Battery_V:.2f}'
    output['btc'] = f'{config_Global_File.batteryUps_Temp_C:5.2f}C'
    output['btf'] = f'{config_Global_File.batteryUps_Temp_F:5.2f}F'

    return json.dumps(output)


if __name__ == '__main__':
    print("*** DEBUG: __main__")

    # jwc 2020-1223 StreamVideoToWebBrowser-AdrianRosebrock
    #
    # construct the argument parser and parse command line arguments
    argumentParser_Cl_Ob = argparse.ArgumentParser()
    ##jwc y argumentParser_Cl_Ob.add_argument("-i", "--ip", type=str, required=True,help="ip address of the device")
    argumentParser_Cl_Ob.add_argument("-a", "--address", type=str, default='0.0.0.0', help="ip address of the device")
    ##jwc y argumentParser_Cl_Ob.add_argument("-o", "--port", type=int, required=True, help="ephemeral port number of the server (1024 to 65535)")
    argumentParser_Cl_Ob.add_argument("-p", "--port", type=int, default=5000, help="ephemeral port number of the server (1024 to 65535)")
    argumentParser_Cl_Ob.add_argument("-f", "--frame-count", type=int, default=32, help="# of frames used to construct the background model")
    # jwc 2.1
    ##jwc o argumentParser_Cl_Ob.add_argument("-thread_Cl_Ob", "--type", type=str, default="DICT_ARUCO_ORIGINAL", help="type of ArUCo tag to detect")
    argumentParser_Cl_Ob.add_argument("-thread_Cl_Ob", "--type", type=str, default="DICT_6X6_100", help="type of ArUCo tag to detect")
    print("*** DEBUG: __main__: --type (default): cv2.aruco.DICT_6X6_100")
    args = vars(argumentParser_Cl_Ob.parse_args())

    # verify that the supplied ArUCo tag exists and is supported by
    # OpenCV
    if ARUCO_DICT.get(args["type"], None) is None:
        print("[INFO] ArUCo tag of '{}' is not supported".format(
            args["type"]))
        sys.exit(0)

    # load the ArUCo dictionary and grab the ArUCo parameters
    print("[INFO] detecting '{}' tags...".format(args["type"]))
    arucoDict = cv2.aruco.Dictionary_get(ARUCO_DICT[args["type"]])
    arucoParams = cv2.aruco.DetectorParameters_create()


    io_Driver_File.light_green_blink(0.1)
    time.sleep(1)
    io_Driver_File.light_green_off()

    # register signal handler for a clean exit    
    signal.signal(signal.SIGINT, signal_handler)

    ##jwc o # register handler for touchpads
    ##jwc o if io_Driver_File.explorerhat:
    ##jwc o     io_Driver_File.xhat.touch.released(touch_handler)
        
    # prepare and start watchdog
    # jwc since watchdog happens so much (seems infinite loop and recursive) and interferes w/ debug, thus turn off
    #
    watchDog = threading.Thread(name='watchdog_timer', target=watchdog_timer)
    watchDog.start()


    # jwc 2020-1223 StreamVideoToWebBrowser-AdrianRosebrock
    #
    # start a thread that will perform motion detection
    ##jwc o AiCam 2.0 thread_Cl_Ob = threading.Thread(target=detect_Motions_Fn, args=(args["frame_count"],))
    ##jwc AiCam 2.1
    ##
    thread_Cl_Ob = threading.Thread(target=detect_Motions_And_ArucoMarkers_Fn, args=(args["frame_count"],))
    thread_Cl_Ob.daemon = True
    thread_Cl_Ob.start()


    ##jwc o app_Cl_Ob.run(host='0.0.0.0', debug=False, threaded=True)
    ##
    ##jwc n app_Cl_Ob.run(host='192.168.1.80', debug=False, threaded=True)
    ##jwc to not conflict with other apps
    ##jwc y app_Cl_Ob.run(host='0.0.0.0', port=5001, debug=False, threaded=True)
    
    ## jwc NameError: name 'WSGIServer' is not defined
    ##jwc on http_server = WSGIServer(('', 5001), app_Cl_Ob)
    ##jwc on http_server.serve_forever()

    ##jwc y app_Cl_Ob.run(host='0.0.0.0', port=5001, debug=False, threaded=True)
    ##jwc n seems to cause rpi crash and video-stream not work: app_Cl_Ob.run(host='0.0.0.0', port=5001, debug=True, threaded=True)
    ##jwc y app_Cl_Ob.run(host='0.0.0.0', port=5001, debug=True, threaded=False)
    ##jwc y app_Cl_Ob.run(host='0.0.0.0', port=5001, debug=True, threaded=True)

    # jwc: Does 'debug=False' prevents two instances of 'main()'
    # jwc: TYJ camera seems to work now, via 'run: start debugging', esp. after rpi reboot

    ##jwc yo app_Cl_Ob.run(host='0.0.0.0', threaded=True)
    ## y app_Cl_Ob.run(host='0.0.0.0', debug=True, threaded=True)
    ##jwc y app_Cl_Ob.run(host='0.0.0.0', threaded=True)

    ##jwc y app_Cl_Ob.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
    ##jwc n app_Cl_Ob.run(host='0.0.0.0', port=80, debug=False, threaded=True)
    ##jwc app_Cl_Ob.run(host='0.0.0.0', port=8888, debug=False, threaded=True)
    ##jwc o app_Cl_Ob.run(host='0.0.0.0', port=5000, debug=False, threaded=True)

    # jwc 2020-1223 StreamVideoToWebBrowser-AdrianRosebrock
    #
    #jwc y app_Cl_Ob.run(host=args["address"], port=args["port"], debug=True, threaded=True, use_reloader=False)
    # jwc TYJ following works with local debugging, though cam may not work
    # jwc o, obsolete since now will use external-VSCode-debugger and not Flask-debugger: app_Cl_Ob.run(host=args["address"], port=args["port"], debug=False, threaded=True, use_reloader=False)
    ## jwc Seems that Flask-dedug now fix and not this 'passthrough_errosrs' workaround: 'passthrough_errors=True' since need errors to bubble up to ext VSCode-debugger
    ##jwc y app_Cl_Ob.run(host=args["address"], port=args["port"], use_debugger=False, threaded=True, use_reloader=False, passthrough_errors=True)
    ##jwc y app_Cl_Ob.run(host=args["address"], port=args["port"], use_debugger=False, threaded=True, use_reloader=False, passthrough_errors=False)
    ##jwc y 2021-0113 app_Cl_Ob.run(host=args["address"], port=args["port"], use_debugger=False, threaded=True, use_reloader=False, passthrough_errors=True)
    
    # * For Flask In-House Development Non-Production Server
    ##jwc yy app_Cl_Ob.run(host=args["address"], port=args["port"], debug=True, threaded=True, use_reloader=False, passthrough_errors=True)
    
    # * For WSGI Server: GUnicorn
    ##jwc n app_Cl_Ob.run()

    # * For WSGI Server: Waitress
    serve( app_Cl_Ob, host='0.0.0.0', port=5000, url_scheme='https' )

# jwc 2020-1223 StreamVideoToWebBrowser-AdrianRosebrock
#
# release the video stream pointer
videoStream_Cl_Ob.stop()