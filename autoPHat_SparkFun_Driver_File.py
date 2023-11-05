#!/usr/bin/env python
#-----------------------------------------------------------------------------
# A simple test to speed up and slow down 1 motor.
#------------------------------------------------------------------------
#
# Written by Mark Lindemer
# SparkFun Electronics, April 2020
# 
# This python library supports the SparkFun Electroncis qwiic 
# qwiic sensor/board ecosystem on a Raspberry Pi (and compatable) single
# board computers. 
#
# More information on qwiic is at https://www.sparkfun.com/qwiic
#
# Do you like this library? Help support SparkFun. Buy a board!
#
#==================================================================================
# Copyright (c) 2019 SparkFun Electronics
#
# Permission is hereby granted, free of charge, to any person obtaining a copy 
# of this software and associated documentation files (the "Software"), to deal 
# in the Software without restriction, including without limitation the rights 
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell 
# copies of the Software, and to permit persons to whom the Software is 
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all 
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE 
# SOFTWARE.
#==================================================================================
# Example 1
#

from __future__ import print_function
import time
import sys
import math
import qwiic_scmd

import pi_servo_hat

# to communicate/command micro:bit via USB-Serial
#

import serial  ## to communicate/command micro:bit

# Open the serial port for sending at speed 115200 bits/second
###jwc undo: device = serial.Serial('/dev/ttyACM0', 115200)
###jwc y microBit_Serial_Obj = serial.Serial('/dev/ttyACM0', 115200)
###jwc y microBit_Serial_Obj = serial.Serial('/dev/ttyACM1', 115200)
###jwc debug y cd: microBit_Serial_Obj = serial.Serial('/dev/ttyAMA0', 115200)
microBit_Serial_Obj = serial.Serial('/dev/ttyACM0', 115200)

# Write string to serial port, and terminate with a line feed character
##jwc y serialString_Tx = "22\n"
##jwc y serialString_Tx = "Hello Jesus :)\n"
##jwc y serialString_Tx = ":)\n"
##jwc ? buffer overload: rpi_Microbit_1CharMsg_Forward_Str = "forward\n"
##jwc ? buffer overload: rpi_Microbit_1CharMsg_Backward_Str = "backward\n"
##jwc ? rpi_Microbit_1CharMsg_Forward_Str = "fo\n"
##jwc ? rpi_Microbit_1CharMsg_Backward_Str = "ba\n"
rpi_Microbit_1CharMsg_Forward_Str = "f\n"
rpi_Microbit_1CharMsg_Backward_Str = "b\n"
rpi_Microbit_1CharMsg_Left_Str = "l\n"
rpi_Microbit_1CharMsg_Right_Str = "r\n"
##jwc y rpi_Microbit_1CharMsg_Stop_Str = "s\n"
rpi_Microbit_1CharMsg_Stop_Str = "s:\n"
rpi_Microbit_1CharMsg_ForwardLeft_Str = "8\n"
rpi_Microbit_1CharMsg_ForwardRight_Str = "9\n"
rpi_Microbit_1CharMsg_ArmForward_Down_Str = "d\n"
rpi_Microbit_1CharMsg_ArmBackward_Up_Str = "u\n"
rpi_Microbit_1CharMsg_Gear1_Str = "1\n"
rpi_Microbit_1CharMsg_Gear2_Str = "2\n"


def init():
    # DC Motors
    #
    global R_MTR, L_MTR, FWD, BWD
    global myMotor

    R_MTR = 0
    L_MTR = 1
    FWD = 0
    BWD = 1

    myMotor = qwiic_scmd.QwiicScmd()

    if myMotor.connected == False:
        print("Motor Driver not connected. Check connections.", \
            file=sys.stderr)
        return
    myMotor.begin()

    # Servos
    #
    global servos_Cam, servo_Cam_01_Pan_ChannelNum, servo_Cam_02_Tilt_ChannelNum, servo_Arm_03_ChannelNum
    servo_Cam_01_Pan_ChannelNum = 0
    servo_Cam_02_Tilt_ChannelNum = 1
    servo_Arm_03_ChannelNum = 2
    # Initialize Constructor
    ##jwc o test = pi_servo_hat.PiServoHat()
    ##jwc direct power, test = pi_servo_hat.PiServoHat(0x48)
    ##jwc y test = pi_servo_hat.PiServoHat()
    ##jwc y test = pi_servo_hat.PiServoHat(0x40)

    ##jwc y servos_Cam = pi_servo_hat.PiServoHat()
    ##jwc n servos_Cam = pi_servo_hat.PiServoHat(0x48, True)
    ##jwc n servos_Cam = pi_servo_hat.PiServoHat(0x41, True)
    ##jwc y servos_Cam = pi_servo_hat.PiServoHat()
    ##jwc n servos_Cam = pi_servo_hat.PiServoHat(0x41)
    servos_Cam = pi_servo_hat.PiServoHat()

    # Restart Servo Hat (in case Hat is frozen/locked)
    servos_Cam.restart()    

    print("Motors & Servos: Setup Done")
    ##jwc o time.sleep(.250)
    # Pause 1 sec
    time.sleep(1)


def go_Tilt_Motor_XDotY_UxSlider_Fn(speed_L_In, speed_R_In):
    ##jwc yn inconsistent rx, maybe too long header?: rpiToMicrobitMsgStr = "ti_mo_xy:{0:03.0f}.{1:03.0f}\n".format(speed_L_In, speed_R_In)
    rpiToMicrobitMsgStr = "xy:{0:03.0f}.{1:03.0f}\n".format(speed_L_In, speed_R_In)
    microBit_Serial_Obj.write(rpiToMicrobitMsgStr.encode())
    print("*** *** *** DEBUG: go_Tilt_Motor_XDotY_UxSlider_Fn: " + rpiToMicrobitMsgStr)

def go_Tilt_Motor_Y_UxSlider_Fn(speed_In):
    ##jwc yn inconsistent rx, maybe too long header?: rpiToMicrobitMsgStr = "ti_mo_xy:{0:03.0f}.{1:03.0f}\n".format(speed_L_In, speed_R_In)
    rpiToMicrobitMsgStr = "y:{0:03.0f}\n".format(speed_In)
    microBit_Serial_Obj.write(rpiToMicrobitMsgStr.encode())
    print("*** *** *** DEBUG: go_Tilt_Motor_Y_UxSlider_Fn: " + rpiToMicrobitMsgStr)

def go_Tilt_Motor_X_UxSlider_Fn(speed_In):
    ##jwc yn inconsistent rx, maybe too long header?: rpiToMicrobitMsgStr = "ti_mo_xy:{0:03.0f}.{1:03.0f}\n".format(speed_L_In, speed_R_In)
    rpiToMicrobitMsgStr = "x:{0:03.0f}\n".format(speed_In)
    microBit_Serial_Obj.write(rpiToMicrobitMsgStr.encode())
    print("*** *** *** DEBUG: go_Tilt_Motor_X_UxSlider_Fn: " + rpiToMicrobitMsgStr)

def go_Forward_UxButton_Fn(speed_In):
    ##jwc y 2021-0519 UsbSerial to micro:bit: myMotor.set_drive(L_MTR,FWD,speed_In)
    ##jwc y 2021-0519 UsbSerial to micro:bit: myMotor.set_drive(R_MTR,FWD,speed_In)
    microBit_Serial_Obj.write(rpi_Microbit_1CharMsg_Forward_Str.encode())
    print("*** *** *** DEBUG: go_Forward_UxButton_Fn: " + rpi_Microbit_1CharMsg_Forward_Str)

def go_Backward_UxButton_Fn(speed_In):
    ##jwc y 2021-0519 UsbSerial to micro:bit: myMotor.set_drive(L_MTR,FWD,speed_In)
    ##jwc y 2021-0519 UsbSerial to micro:bit: myMotor.set_drive(R_MTR,FWD,speed_In)
    microBit_Serial_Obj.write(rpi_Microbit_1CharMsg_Backward_Str.encode())
    print("*** *** *** DEBUG: go_Backward_UxButton_Fn: " + rpi_Microbit_1CharMsg_Backward_Str)

def go_Left_UxButton_Fn(speed_In):
    microBit_Serial_Obj.write(rpi_Microbit_1CharMsg_Left_Str.encode())
    print("*** *** *** DEBUG: go_Left_UxButton_Fn: " + rpi_Microbit_1CharMsg_Left_Str)

def go_Right_UxButton_Fn(speed_In):
    microBit_Serial_Obj.write(rpi_Microbit_1CharMsg_Right_Str.encode())
    print("*** *** *** DEBUG: go_Right_UxButton_Fn: " + rpi_Microbit_1CharMsg_Right_Str)

def go_Stop_UxButton_Fn(speed_In):
    microBit_Serial_Obj.write(rpi_Microbit_1CharMsg_Stop_Str.encode())
    print("*** *** *** DEBUG: go_Stop_UxButton_Fn: " + rpi_Microbit_1CharMsg_Stop_Str)


def motorLeft_Fn(speed_In):
    myMotor.set_drive(L_MTR,FWD,speed_In)

def motorRight_Fn(speed_In):
    myMotor.set_drive(R_MTR,FWD,speed_In)
    
def motorStop_Fn(speed_In):
    myMotor.set_drive(L_MTR,FWD,speed_In)
    myMotor.set_drive(R_MTR,FWD,speed_In)


def servo_Cam_01_Pan_Fn(degrees_In):
    servos_Cam.move_servo_position(servo_Cam_01_Pan_ChannelNum, degrees_In, 180)

def servo_Cam_02_Tilt_Fn(degrees_In):
    servos_Cam.move_servo_position(servo_Cam_02_Tilt_ChannelNum, degrees_In, 180)

def servo_Arm_03_Fn(degrees_In):
    servos_Cam.move_servo_position(servo_Arm_03_ChannelNum, degrees_In, 180)

# jwc Returns unusual value.  Do not use for now.
def servo_Cam_01_Pan_PositionGet_Fn():
    return servos_Cam.get_servo_position(servo_Cam_01_Pan_ChannelNum)

# jwc Returns unusual value.  Do not use for now.
def servo_Cam_02_Tilt_PositionGet_Fn():
    return servos_Cam.get_servo_position(servo_Cam_02_Tilt_ChannelNum)


def runExample():
    print("Motor Test.")

    # Zero Motor Speeds
    myMotor.set_drive(0,0,0)
    myMotor.set_drive(1,0,0)

    myMotor.enable()
    print("Motor enabled")
    time.sleep(.250)


    while True:
        speed = 20
        for speed in range(20,255):
            print(speed)
            myMotor.set_drive(R_MTR,FWD,speed)
            myMotor.set_drive(L_MTR,BWD,speed)
            time.sleep(.05)
        for speed in range(254,20,-1):
            print(speed)
            myMotor.set_drive(R_MTR,FWD,speed)
            myMotor.set_drive(L_MTR,BWD,speed)
            time.sleep(.05)

def runTest():
    print("Motor Test.")

    # Test Run
    #########################################
    # Moves servo channel 0, position to 90 degrees (1ms), swing max 180
    servos_Cam.move_servo_position(servo_Cam_01_Pan_ChannelNum, 0, 180)
    time.sleep(3)
    servos_Cam.move_servo_position(servo_Cam_01_Pan_ChannelNum, 90, 180)
    time.sleep(3)
    servos_Cam.move_servo_position(servo_Cam_01_Pan_ChannelNum, 180, 180)
    time.sleep(3)
    # Moves servo channel 1, position to 90 degrees (1ms), swing max 180
    servos_Cam.move_servo_position(servo_Cam_02_Tilt_ChannelNum, 0, 180)
    time.sleep(3)
    servos_Cam.move_servo_position(servo_Cam_02_Tilt_ChannelNum, 90, 180)
    time.sleep(3)
    servos_Cam.move_servo_position(servo_Cam_02_Tilt_ChannelNum, 180, 180)
    time.sleep(3)

    # Zero Motor Speeds
    myMotor.set_drive(0,0,0)
    myMotor.set_drive(1,0,0)

    myMotor.enable()
    print("Motor enabled")
    time.sleep(.250)

    ##jwc o while True:
    speed = 20
    for speed in range(20,255):
        print(speed)
        myMotor.set_drive(R_MTR,FWD,speed)
        myMotor.set_drive(L_MTR,BWD,speed)
        ##jwc o time.sleep(.05)
        time.sleep(.05)
    for speed in range(254,20,-1):
        print(speed)
        myMotor.set_drive(R_MTR,FWD,speed)
        myMotor.set_drive(L_MTR,BWD,speed)
        ##jwc o time.sleep(.05)
        time.sleep(.05)


def runTest_Quick():
    print("Motor Test.")

    # Test Run
    #########################################
    # Moves servo channel 0, position to 90 degrees (1ms), swing max 180
    servos_Cam.move_servo_position(servo_Cam_01_Pan_ChannelNum, 0, 180)
    time.sleep(1)
    servos_Cam.move_servo_position(servo_Cam_01_Pan_ChannelNum, 90, 180)
    time.sleep(1)
    servos_Cam.move_servo_position(servo_Cam_01_Pan_ChannelNum, 180, 180)
    time.sleep(1)
    # Moves servo channel 1, position to 90 degrees (1ms), swing max 180
    servos_Cam.move_servo_position(servo_Cam_02_Tilt_ChannelNum, 0, 180)
    time.sleep(1)
    servos_Cam.move_servo_position(servo_Cam_02_Tilt_ChannelNum, 90, 180)
    time.sleep(1)
    servos_Cam.move_servo_position(servo_Cam_02_Tilt_ChannelNum, 180, 180)
    time.sleep(1)
    # Moves servo channel 2, position to 90 degrees (1ms), swing max 180
    servos_Cam.move_servo_position(servo_Arm_03_ChannelNum, 0, 180)
    time.sleep(1)
    servos_Cam.move_servo_position(servo_Arm_03_ChannelNum, 90, 180)
    time.sleep(1)
    servos_Cam.move_servo_position(servo_Arm_03_ChannelNum, 180, 180)
    time.sleep(1)

    myMotor.enable()
    print("Motor enabled")
    time.sleep(.250)

    ##jwc o while True:
    ##jwc o  for speed in range(20,255):
    # Range-End is exclusive
    for speed in range(154,264,10):
        print(speed)
        myMotor.set_drive(R_MTR,FWD,speed)
        myMotor.set_drive(L_MTR,BWD,speed)
        ##jwc o time.sleep(.05)
        time.sleep(.05)
    ##jwc o for speed in range(254,20,-1):
    # Range-End is exclusive
    for speed in range(254,144,-10):
        print(speed)
        myMotor.set_drive(R_MTR,FWD,speed)
        myMotor.set_drive(L_MTR,BWD,speed)
        ##jwc o time.sleep(.05)
        time.sleep(.05)

    # Zero Motor Speeds
    myMotor.set_drive(0,0,0)
    myMotor.set_drive(1,0,0)


if __name__ == '__main__':
    try:
        runExample()
    except (KeyboardInterrupt, SystemExit) as exErr:
        print("Ending example.")
        myMotor.disable()
        sys.exit(0)
