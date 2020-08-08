#!/usr/bin/env python3
##jwc o #!/usr/bin/env python

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


import RPi.GPIO as GPIO
from adafruit_crickit import crickit

# make two variables for the motors to make code shorter to type
# Right-Side
motor_1 = crickit.dc_motor_1
# Left-Side
motor_2 = crickit.dc_motor_2

servo_1 = crickit.servo_1
servo_2 = crickit.servo_2

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# import camera driver
if os.environ.get('CAMERA'):
    Camera = import_module('camera_' + os.environ['CAMERA']).Camera
else:
    ##jwc o from camera import Camera
    # Default to most sophisticated tech
    from camera_opencv import Camera

# Raspberry Pi camera module (requires picamera package)
# from camera_pi import Camera

import logging
log = logging.getLogger('werkzeug')
##jwc o log.setLevel(logging.ERROR)
log.setLevel(logging.INFO)

##jwc yo app = Flask(__name__)

app = Flask(__name__, static_url_path='/static')


# Immobilizes sytem (chocks on) after 'timeout' seconds 
def watchdog_timer():
    while cfg.watchdog_Alive_Bool:
        # jwc: Pauses every 1sec
        ##jwc o time.sleep(1)
        ##jwc
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
        cfg.blue = not cfg.blue
        if cfg.blue:
            hw.light_blue_on()
            hw.output_one_on()
        else:
            hw.light_blue_off()
            hw.output_one_off()
    if channel == 2:
        cfg.yellow = not cfg.yellow
        if cfg.yellow:
            hw.light_yellow_on()
            hw.output_two_on()
        else:
            hw.light_yellow_off()
            hw.output_two_off()
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
        cfg.green = True
        time.sleep(5)
        if cfg.chocks:
            hw.light_green_on()
            ##jwc o os.system("sudo -s shutdown -h now")
        else:
            hw.light_green_off()
            cfg.green = False
    
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


@app.route('/')
def index():
    """Video streaming home page."""
    return render_template('index.html')

def gen(camera):
    """Video streaming generator function."""
    while True:
        frame = camera.get_frame()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

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

            print("motor-left: " + str(left) + " " + str(left_normalized))
    return 'ok'
 """
# URL for motor control - format: /motor?l=[speed]&r=[speed]
@app.route('/motor')
def motor():
    left = request.args.get('l')
    right = request.args.get('r')
    left_normalized = 0
    right_normalized = 0

    if left and not cfg.chocks:
        left = int(left)
        if left >= -100 and left <= 100:
            ##jwc yo cfg.left_motor = left
            ##jwc o hw.motor_two_speed(cfg.left_motor)
            left_normalized = (left / 100 )
            motor_1.throttle = left_normalized

    if right and not cfg.chocks:
        right = int(right)
        if right >= -100 and right <= 100:
            ##jwc o cfg.right_motor = right
            ##jwc o hw.motor_one_speed(cfg.right_motor)
            right_normalized = (right / 100 )
            motor_2.throttle = right_normalized

    print("*** DEBUG: motor: l" + str(left_normalized) + " r" + str(right_normalized))
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

@app.route('/video_feed')
def video_feed():
    """Video streaming route. Put this in the src attribute of an img tag."""
    return Response(gen(Camera()), mimetype='multipart/x-mixed-replace; boundary=frame')


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
    
    ##jwc o output['l'] = cfg.left_motor
    output['l'] = motor_1.throttle
    ##jwc o output['r'] = cfg.right_motor
    output['r'] = motor_2.throttle

    output['i1'] = hw.input_one_read()
    output['i2'] = hw.input_two_read()
    output['i3'] = hw.input_three_read()
    output['i4'] = hw.input_four_read()
    ##jwc o output['a1'] = hw.analog_one_read()
    ##jwc o output['a2'] = hw.analog_two_read()
    ##jwc o output['a3'] = hw.analog_three_read()
    ##jwc o output['a4'] = hw.analog_four_read()
    return json.dumps(output)



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

