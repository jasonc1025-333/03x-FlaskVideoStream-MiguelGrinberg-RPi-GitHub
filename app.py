
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

import config as cfg

##jwc o import io_wrapper as hw
import io_wrapper_dummy as hw

## jwc o import RPi.GPIO as GPIO

##jwc y from gpiozero import Servo

##jwc o from adafruit_crickit import crickit
##jwc y import robohat
##jwc y robohat.init()

import AutoPHat_SparkFun_Driver

AutoPHat_SparkFun_Driver.init()
##jwc y AutoPHat_SparkFun_Driver.runTest()
AutoPHat_SparkFun_Driver.runTest_Quick()

##jwc n global servo_01_Pan_Degrees 
##jwc n servo_01_Pan_Degrees = 90
AutoPHat_SparkFun_Driver.servo_Cam_01_Pan_Fn( cfg.servo_01_Pan_Degrees )
AutoPHat_SparkFun_Driver.servo_Cam_02_Tilt_Fn( cfg.servo_02_Tilt_Degrees )

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


# import camera driver
if os.environ.get('CAMERA'):
    Camera = import_module('camera_' + os.environ['CAMERA']).Camera
    print("*** DEBUG: Camera-01: camera_" + os.environ['CAMERA'])
else:
    ##jwc o from camera import Camera
    # Default to most sophisticated tech
    from camera_opencv import Camera
    print("*** DEBUG: Camera-02: camera_opencv")

# Raspberry Pi camera module (requires picamera package)
# from camera_pi import Camera

import logging
log = logging.getLogger('werkzeug')
##jwc o log.setLevel(logging.ERROR)
log.setLevel(logging.INFO)


# jwc ~/01-Jwc/2020-1205-0640-RpiRover1-DayDevelops/RpiRover1-master/
# jwc 'sys.path.append('db') does work, depsite 'lint' not understand for 'import'
sys.path.append('db')
import robotProperties_Db_Cl_File

DB = robotProperties_Db_Cl_File.robotProperties_Db_Cl()
trims = DB.getTrimValues()
print("*** DEBUG: DB: " + json.dumps( trims ))
cfg.left_motor_trim = trims['L']
cfg.right_motor_trim = trims['R']
print ("*** DEBUG: DB: cfg.left_motor_trim: " + str( cfg.left_motor_trim ) + ", cfg.right_motor_trim: " + str( cfg.right_motor_trim ))


##jwc yo app = Flask(__name__)

app = Flask(__name__, static_url_path='/static')

    
@app.route('/')
def index():
    """Video streaming home page."""
    return render_template('index.html')


# Immobilizes sytem (chocks on) after 'timeout' seconds 
def watchdog_timer():
    while cfg.watchdog_Alive_Bool:
        # jwc: Pauses every 1sec
        ##jwc o time.sleep(1)
        ##jwc
        ##jwc y  decrease for more real-time stats: time.sleep(5)
        ##jwc still too long: time.sleep(1)
        ##jwc time.sleep(.10)
        ##jwc y  realize this not affect HUD Stats, so keep as is to prevent premature disconnect
        time.sleep(5)
        
        if cfg.watchdog_Start_On_Bool:
            cfg.watchdog_Cycles_Now += 1
            ##jwc print("*** DEBUG: cfg.watchdog_Cycles_Now: " + str(cfg.watchdog_Cycles_Now))
            # jwc: appears that beyond 10sec is bad disconnect, 
            #      so engage 'chocks/disable bot', if not already
            if cfg.watchdog_Cycles_Now > cfg.timeout_Cycles_MAX and not cfg.chocks:
                chocks_on()
            if cfg.watchdog_Cycles_Now <= cfg.timeout_Cycles_MAX and cfg.chocks:
                chocks_off()

# Handler for a clean shutdown when pressing Ctrl-C
def signal_handler(signal, frame):
    hw.light_blue_blink(0.1)
    cfg.watchdog_Alive_Bool = False
    cfg.camera_active = False
    brakes_on()
    # jwc: Wait until thread terminates
    watchDog.join()
    ##jwc o http_server.close()
    hw.light_blue_off()
    sys.exit(0)

# Handler for explorer-hat touchpads
def touch_handler(channel, event):

    if channel == 1:
        #jwc o 
        # cfg.blue = not cfg.blue
        # if cfg.blue:
        #     hw.light_blue_on()
        #     hw.output_one_on()
        # else:
        #     hw.light_blue_off()
        #     hw.output_one_off()

        ##jwc n AttributeError: servo_01_Pan = AutoPHat_SparkFun_Driver.servo_Cam_01_Pan_PositiionGet_Fn()
        ##jwc n cfg.servo_01_Pan_Degrees = AutoPHat_SparkFun_Driver.servo_Cam_01_Pan_PositionGet_Fn()

        cfg.servo_01_Pan_Degrees = cfg.servo_01_Pan_Degrees - 10
        if cfg.servo_01_Pan_Degrees < 0: 
            cfg.servo_01_Pan_Degrees = 0
        AutoPHat_SparkFun_Driver.servo_Cam_01_Pan_Fn( cfg.servo_01_Pan_Degrees )
        print("*** DEBUG: S1: servo: pan: " + str(cfg.servo_01_Pan_Degrees))
        
    if channel == 2:
        #jwc o 
        # cfg.yellow = not cfg.yellow
        # if cfg.yellow:
        #     hw.light_yellow_on()
        #     hw.output_two_on()
        # else:
        #     hw.light_yellow_off()
        #     hw.output_two_off()

        ##jwc n AttributeError: servo_01_Pan = AutoPHat_SparkFun_Driver.servo_Cam_01_Pan_PositiionGet_Fn()
        ##jwc n cfg.servo_01_Pan_Degrees = AutoPHat_SparkFun_Driver.servo_Cam_01_Pan_PositionGet_Fn()

        cfg.servo_01_Pan_Degrees = cfg.servo_01_Pan_Degrees + 10
        if cfg.servo_01_Pan_Degrees > 180: 
            cfg.servo_01_Pan_Degrees = 180
        AutoPHat_SparkFun_Driver.servo_Cam_01_Pan_Fn( cfg.servo_01_Pan_Degrees )
        print("*** DEBUG: S1: servo: pan: " + str(cfg.servo_01_Pan_Degrees))

    if channel == 3:
        cfg.chocks = not cfg.chocks
        # jwc: Chocks set to True: Admin Lock
        if cfg.chocks:
            # jwc: Since Motors not free to operate, Watchdog not needed
            cfg.watchdog_Start_On_Bool = False
            chocks_on()
        # jwc: Chocks set to False: Admin Unlock
        else:
            # jwc: Since Motors are free to operate, Watchdog is needed
            cfg.watchdog_Start_On_Bool = True
            chocks_off()

    if channel == 4:
        hw.light_green_blink(0.1)
        #jwc o 
        # cfg.green = True
        # ##jwc o time.sleep(5)
        # if cfg.chocks:
        #     hw.light_green_on()
        #     ##jwc o os.system("sudo -s shutdown -h now")
        # else:
        #     hw.light_green_off()
        #     cfg.green = False

        ##jwc n cfg.temp = AutoPHat_SparkFun_Driver.servo_Cam_02_Tilt_PositionGet_Fn()

        ##jwc n
        # servoCam02Tilt = servoCam02Tilt + 10
        # AutoPHat_SparkFun_Driver.servo_Cam_02_Tilt_Fn( servoCam02Tilt ) 
        # print("*** DEBUG: S2: servo: tilt: " + str(AutoPHat_SparkFun_Driver.servo_Cam_02_Tilt_PositionGet_Fn()))

        cfg.servo_02_Tilt_Degrees = cfg.servo_02_Tilt_Degrees - 10
        if cfg.servo_02_Tilt_Degrees < 0: 
            cfg.servo_02_Tilt_Degrees = 0
        AutoPHat_SparkFun_Driver.servo_Cam_02_Tilt_Fn( cfg.servo_02_Tilt_Degrees )
        print("*** DEBUG: S2: servo: tilt: " + str(cfg.servo_02_Tilt_Degrees))

    if channel == 5:
        hw.light_green_blink(0.1)
        #jwc o 
        # cfg.green = True
        # ##jwc o time.sleep(5)
        # if cfg.chocks:
        #     hw.light_green_on()
        #     ##jwc o os.system("sudo -s shutdown -h now")
        # else:
        #     hw.light_green_off()
        #     cfg.green = False

        ##jwc n cfg.temp = AutoPHat_SparkFun_Driver.servo_Cam_02_Tilt_PositionGet_Fn()

        ##jwc n
        # servoCam02Tilt = servoCam02Tilt + 10
        # AutoPHat_SparkFun_Driver.servo_Cam_02_Tilt_Fn( servoCam02Tilt ) 
        # print("*** DEBUG: S2: servo: tilt: " + str(AutoPHat_SparkFun_Driver.servo_Cam_02_Tilt_PositionGet_Fn()))

        cfg.servo_02_Tilt_Degrees = cfg.servo_02_Tilt_Degrees + 10
        if cfg.servo_02_Tilt_Degrees > 180: 
            cfg.servo_02_Tilt_Degrees = 180
        AutoPHat_SparkFun_Driver.servo_Cam_02_Tilt_Fn( cfg.servo_02_Tilt_Degrees )
        print("*** DEBUG: S2: servo: tilt: " + str(cfg.servo_02_Tilt_Degrees))

def brakes_on():
    cfg.brakes = True
    cfg.left_motor = 0
    cfg.right_motor = 0
    ##jwc o hw.motor_one_speed(cfg.right_motor)
    ##jwc o hw.motor_two_speed(cfg.left_motor)
    
# jwc: Motors free to operate: Lo-Level: User-Level
def brakes_off():
    cfg.brakes = False
    cfg.watchdog_Cycles_Now = 0
    
def chocks_on():
    cfg.chocks = True
    brakes_on()
    hw.light_red_blink(0.2)

# jwc: Motors free to operate: Hi-Level: Admin-Level ~ Overrides User-Level for Security/Safety
def chocks_off():
    cfg.chocks = False
    brakes_off()
    hw.light_red_off()


""" jwc y
# URL for motor control - format: /motor?l=[speed]&r=[speed]
@app.route('/motor')
def motor():
    left = request.args.get('l')
    ##o if left and not cfg.chocks:
    if left:
        left = int(left)
        if left >= -100 and left <= 100:
            ##o cfg.left_motor = left
            ##o hw.motor_two_speed(cfg.left_motor)
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
@app.route('/motor')
def motor():
    left = request.args.get('l')
    right = request.args.get('r')

    print("*** *** DEBUG: left: " + str(left) + " right: " + str(right))

    left_normalized = 0
    right_normalized = 0

    if left and not cfg.chocks:
        left_Int = int(left)
        cfg.left_motor = left_Int
        left_Absolute = abs( left_Int )
        if left_Int >= -100 and left_Int <= 100:
            ##jwc yo cfg.left_motor = left
            ##jwc o hw.motor_two_speed(cfg.left_motor)
            ##jwc o left_normalized = (left / 100 )
            if left_Int >= 0:
                ##jwc n robohat.motorLeft_Plus_Fn(left_Absolute)
                ##jwc y robohat.motorLeft_Plus_Fn(left_Absolute)
                ##jwc y AutoPHat_SparkFun_Driver.motorLeft_Fn( left_Int )
                AutoPHat_SparkFun_Driver.motorLeft_Fn( (left_Int/100) * 250 )
                ##jwc n AutoPHat_SparkFun_Driver.motorRight_Fn( (left_Int/100) * 250 )

                # Sample Test Servo
                ##TODO jwc jittery:  servo.min()
                ##TODO jwc jitter: servo.ChangeDutyCycle(servoPwm_PositionMax)
                ##jwc 'crickit' y servo_02.ChangeDutyCycle(servoPwm_PositionMax)
                ##jwc y servo.max()
                ##jwc y AutoPHat_SparkFun_Driver.servo_Cam_01_Pan_Fn( 0 )
                print("*** DEBUG: L1: motor: L " + str((left_Int/100) * 250))
            elif left_Int < 0:
                ##jwc y robohat.motorLeft_Minus_Fn(left_Absolute)
                ##jwc y AutoPHat_SparkFun_Driver.motorLeft_Fn( left_Int )
                AutoPHat_SparkFun_Driver.motorLeft_Fn( (left_Int/100) * 250 )
                ##jwc n AutoPHat_SparkFun_Driver.motorRight_Fn( (left_Int/100) * 250 )

                # Sample Test Servo
                ##TODO jwc jittery: servo.max()
                ##TODO jwc jitter: servo.ChangeDutyCycle(servoPwm_PositionMin)
                ##jwc 'crickit' y servo_02.ChangeDutyCycle(servoPwm_PositionMin)
                ##jwc y servo.min()
                ##jwc y AutoPHat_SparkFun_Driver.servo_Cam_01_Pan_Fn( 180 )
                print("*** DEBUG: L2: motor: L " + str((left_Int/100) * 250))
            else:
                print("*** Error: Invalid Value: left_Int: ", left_Int)
            ##jwc o motor_1.throttle = left_normalized

    if right and not cfg.chocks:
        right_Int = int(right)
        cfg.right_motor = right_Int
        right_Absolute = abs( right_Int )
        if right_Int >= -100 and right_Int <= 100:
            ##jwc o cfg.right_motor = right
            ##jwc o hw.motor_one_speed(cfg.right_motor)
            ##jwc o right_normalized = (right / 100 )
            if right_Int >= 0:
                ##jwc y robohat.motorRight_Plus_Fn(right_Absolute)
                ##jwc y AutoPHat_SparkFun_Driver.motorRight_Fn( right_Int )
                AutoPHat_SparkFun_Driver.motorRight_Fn( (right_Int/100) * 250 )
                ##jwc n AutoPHat_SparkFun_Driver.motorRight_Fn( -1 * (right_Int/100) * 250 )

                # Sample Test Servo
                ##TODO jwc jittery: servo_02.min()
                ##TODO jwc jitter: servo.ChangeDutyCycle(servoPwm_PositionMax)
                ##jwc 'crickit' y servo_02.ChangeDutyCycle(servoPwm_PositionMax)
                ##jwc y servo_02.max()
                ##jwc y AutoPHat_SparkFun_Driver.servo_Cam_02_Tilt_Fn( 0 )
                print("*** DEBUG: R1: motor: R " + str((right_Int/100) * 250))
            elif right_Int < 0:
                ##jwc y robohat.motorRight_Plus_Fn(right_Absolute)
                ##jwc y AutoPHat_SparkFun_Driver.motorRight_Fn( right_Int )
                AutoPHat_SparkFun_Driver.motorRight_Fn( (right_Int/100) * 250 )
                ##jwc n AutoPHat_SparkFun_Driver.motorRight_Fn( -1 * (right_Int/100) * 250 )

                # Sample Test Servo
                ##TODO jwc jittery: servo_02.max()
                ##TODO jwc jittery: servo.ChangeDutyCycle(servoPwm_PositionMin)
                ##jwc 'crickit' y servo_02.ChangeDutyCycle(servoPwm_PositionMin)
                ##jwc y servo_02.max()
                ##jwc y AutoPHat_SparkFun_Driver.servo_Cam_02_Tilt_Fn( 180 )
                print("*** DEBUG: R2: motor: R " + str((right_Int/100) * 250))
            else:
                print("*** Error: Invalid Value: right_Int: ", right_Int)
            ##jwc o motor_2.throttle = right_normalized

    ##jwc y print("*** DEBUG: motor: l" + str(left_normalized) + " r" + str(right_normalized))
    ##jwc y print("*** DEBUG: motor: l" + str(left_Int) + " r" + str(right_Int)
    ##jwc yn print("*** DEBUG: motor: l" + str((left_Int/100) * 250) + " r" + str((right_Int/100) * 250))
    return 'ok'


# URL for motor control - format: /motor?l=[speed]&r=[speed]
@app.route('/motorTrim')
def motorTrim():
    left = request.args.get('l')
    right = request.args.get('r')

    print("*** *** DEBUG: motorTrim() Pre : left: " + str(left) + " right: " + str(right))

    cfg.left_motor_trim += int( left )
    cfg.right_motor_trim += int( right )

    DB.updateTrimValues(cfg.left_motor_trim, cfg.right_motor_trim)

    print("*** *** DEBUG: motorTrim() Post: left: " + str(cfg.left_motor_trim) + " right: " + str(cfg.right_motor_trim))

    return 'ok'


""" jwc o
 # URL for joystick input - format: /joystick?x=[x-axis]&y=[y-axis]
@app.route('/joystick')
def joystick():
    cfg.watchdog_Cycles_Now = 0
    x_axis = int(request.args.get('x'))
    y_axis = int(request.args.get('y'))
    x_axis = -1 * max( min(x_axis, 100), -100)
    y_axis = max( min(y_axis, 100), -100)
    v = (100-abs(x_axis)) * (y_axis/100) + y_axis
    w = (100-abs(y_axis)) * (x_axis/100) + x_axis
    r = int((v+w) / 2)
    l = int((v-w) / 2)
    if not cfg.chocks:
        cfg.right_motor = r
        cfg.left_motor = l
        hw.motor_one_speed(cfg.right_motor)
        hw.motor_two_speed(cfg.left_motor)
    return 'ok'
 """

# URL to remote control touchpads 1-4 on explorer-hat
@app.route('/touchpad')
def touchpad():
    pad = request.args.get('pad')
    if pad:
        touch_handler(int(pad), True)
    return 'ok'

# URL for heartbeat requests (resets watchdog timer)    
# Returns JSON object with status data
@app.route('/heartbeat')
def heartbeat():
    cfg.watchdog_Cycles_Now = 0
    output = {}
    output['b'] = cfg.blue
    output['y'] = cfg.yellow
    output['c'] = cfg.chocks
    output['g'] = cfg.green
    output['f'] = cfg.video_fps
    output['v'] = cfg.video_status
    
    output['l'] = cfg.left_motor
    ##jwc o output['l'] = motor_1.throttle
    output['r'] = cfg.right_motor
    ##jwc o output['r'] = motor_2.throttle

    # jwc 
    # 
    output['lt'] = cfg.left_motor_trim
    output['rt'] = cfg.right_motor_trim

    output['s1'] = cfg.servo_01_Pan_Degrees
    output['s2'] = cfg.servo_02_Tilt_Degrees
    output['s3'] = cfg.servo_03_Degrees
    output['s4'] = cfg.servo_04_Degrees

    output['i1'] = hw.input_one_read()
    output['i2'] = hw.input_two_read()
    output['i3'] = hw.input_three_read()
    output['i4'] = hw.input_four_read()
    ##jwc o output['a1'] = hw.analog_one_read()
    ##jwc o output['a2'] = hw.analog_two_read()
    ##jwc o output['a3'] = hw.analog_three_read()
    ##jwc o output['a4'] = hw.analog_four_read()
    return json.dumps(output)


def gen(camera):
    """Video streaming generator function."""
    while True:
        frame = camera.get_frame()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

               
@app.route('/video_feed')
def video_feed():
    """Video streaming route. Put this in the src attribute of an img tag."""
    return Response(gen(Camera()), mimetype='multipart/x-mixed-replace; boundary=frame')


if __name__ == '__main__':
    print("*** DEBUG: __main__")

    hw.light_green_blink(0.1)
    time.sleep(1)
    hw.light_green_off()

    # register signal handler for a clean exit    
    signal.signal(signal.SIGINT, signal_handler)

    ##jwc o # register handler for touchpads
    ##jwc o if hw.explorerhat:
    ##jwc o     hw.xhat.touch.released(touch_handler)
        
    # prepare and start watchdog
    # jwc since watchdog happens so much (seems infinite loop and recursive) and interferes w/ debug, thus turn off
    #
    watchDog = threading.Thread(name='watchdog_timer', target=watchdog_timer)
    watchDog.start()

    ##jwc o app.run(host='0.0.0.0', debug=False, threaded=True)
    ##
    ##jwc n app.run(host='192.168.1.80', debug=False, threaded=True)
    ##jwc to not conflict with other apps
    ##jwc y app.run(host='0.0.0.0', port=5001, debug=False, threaded=True)
    
    ## jwc NameError: name 'WSGIServer' is not defined
    ##jwc on http_server = WSGIServer(('', 5001), app)
    ##jwc on http_server.serve_forever()

    ##jwc y app.run(host='0.0.0.0', port=5001, debug=False, threaded=True)
    ##jwc n seems to cause rpi crash and video-stream not work: app.run(host='0.0.0.0', port=5001, debug=True, threaded=True)
    ##jwc y app.run(host='0.0.0.0', port=5001, debug=True, threaded=False)
    ##jwc y app.run(host='0.0.0.0', port=5001, debug=True, threaded=True)

    # jwc: Does 'debug=False' prevents two instances of 'main()'
    # jwc: TYJ camera seems to work now, via 'run: start debugging', esp. after rpi reboot


    ##jwc yo app.run(host='0.0.0.0', threaded=True)
    ## y app.run(host='0.0.0.0', debug=True, threaded=True)
    ##jwc y app.run(host='0.0.0.0', threaded=True)

    ##jwc y app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
    ##jwc n app.run(host='0.0.0.0', port=80, debug=False, threaded=True)
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
    ##jwc app.run(host='0.0.0.0', port=8888, debug=False, threaded=True)

