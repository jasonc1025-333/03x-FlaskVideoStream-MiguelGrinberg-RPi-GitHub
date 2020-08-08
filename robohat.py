#!/usr/bin/python
#
# Python Module to externalise all Initio/RoboHAT specific hardware
#
# Created by Gareth Davies, Feb 2016
# Copyright 4tronix
#
# This code is in the public domain and may be freely copied and used
# No warranty is provided or implied
#
#======================================================================


#======================================================================
# General Functions
#
# init(). Initialises GPIO pins, switches motors off, etc
# cleanup(). Sets all motors off and sets GPIO to standard values
# version(). Returns 2. Invalid until after init() has been called
#======================================================================


#======================================================================
# Motor Functions
#
# stop(): Stops both motors
# forward(speed): Sets both motors to move forward at speed. 0 <= speed <= 100
# reverse(speed): Sets both motors to reverse at speed. 0 <= speed <= 100
# spinLeft(speed): Sets motors to turn opposite directions at speed. 0 <= speed <= 100
# spinRight(speed): Sets motors to turn opposite directions at speed. 0 <= speed <= 100
# turnForward(leftSpeed, rightSpeed): Moves forwards in an arc by setting different speeds. 0 <= leftSpeed,rightSpeed <= 100
# turnreverse(leftSpeed, rightSpeed): Moves backwards in an arc by setting different speeds. 0 <= leftSpeed,rightSpeed <= 100
#======================================================================


#======================================================================
# IR Sensor Functions
#
# irLeft(): Returns state of Left IR Obstacle sensor
# irRight(): Returns state of Right IR Obstacle sensor
# irAll(): Returns true if either of the Obstacle sensors are triggered
# irLeftLine(): Returns state of Left IR Line sensor
# irRightLine(): Returns state of Right IR Line sensor
#======================================================================


#======================================================================
# UltraSonic Functions
#
# getDistance(). Returns the distance in cm to the nearest reflecting object. 0 == no object
#======================================================================

#======================================================================
# Servo Functions
# 
# startServos(). Initialises the servo background process
# stop Servos(). terminates the servo background process
# setServo(Servo, Degrees). Sets the servo to position in degrees -90 to +90
#======================================================================


# Import all necessary libraries
import RPi.GPIO as GPIO, sys, threading, time, os, subprocess

# Pins 35, 36 Left Motor
# Pins 32, 33 Right Motor
##jwc o Board#: L1 = 36
##jwc o Board#: L2 = 35
##jwc o Board#: R1 = 33
##jwc o Board#: R2 = 32

# jwc: Convert from Board# to BCM#
##jwc n BCM#: L1 = 16
##jwc n BCM#: L2 = 19
##jwc n BCM#: R1 = 13
##jwc n BCM#: R2 = 12
L1 = 16
L2 = 19
R1 = 13
R2 = 12

# Define obstacle sensors and line sensors
# These can be on any input pins, but this library assumes the following layout
# which matches the build instructions
##jwc o Board#: irFL = 7
##jwc o Board#: irFR = 11
##jwc o Board#: lineLeft = 29
##jwc o Board#: lineRight = 13

#jwc in BCM#
#
irFL = 4
irFR = 17
lineLeft = 5
lineRight = 27
# Define Sonar Pin (Uses same pin for both Ping and Echo)
##jwc o Board#: sonar = 38

#jwc BCM#
sonar = 20

ServosActive = False

#======================================================================
# General Functions
#
# init(). Initialises GPIO pins, switches motors and LEDs Off, etc
def init():
    global pinLeft_Plus, pinLeft_Minus, pinRight_Plus, pinRight_Minus

    GPIO.setwarnings(False)
    
    #use physical pin numbering
    ##jwc o GPIO.setmode(GPIO.BOARD)
    #jwc use BCM mode since GpioZero based on BCM
    GPIO.setmode(GPIO.BCM)

    #print GPIO.RPI_REVISION

    #set up digital line detectors as inputs
    GPIO.setup(lineRight, GPIO.IN) # Right line sensor
    GPIO.setup(lineLeft, GPIO.IN) # Left line sensor

    #Set up IR obstacle sensors as inputs
    GPIO.setup(irFL, GPIO.IN) # Left obstacle sensor
    GPIO.setup(irFR, GPIO.IN) # Right obstacle sensor

    #use pwm on inputs so motors don't go too fast
    # jwc: 5ms = 0.005s -> 1 / 200 Hz = 200Hz
    # 20 Hz = 1 / 20 Hz = 0.05 sec = 50 msec period
    #

    GPIO.setup(L1, GPIO.OUT)
    pinLeft_Plus = GPIO.PWM(L1, 20)
    pinLeft_Plus.start(0)

    GPIO.setup(L2, GPIO.OUT)
    pinLeft_Minus = GPIO.PWM(L2, 20)
    pinLeft_Minus.start(0)

    GPIO.setup(R1, GPIO.OUT)
    pinRight_Plus = GPIO.PWM(R1, 20)
    pinRight_Plus.start(0)

    GPIO.setup(R2, GPIO.OUT)
    pinRight_Minus = GPIO.PWM(R2, 20)
    pinRight_Minus.start(0)

    startServos()

# cleanup(). Sets all motors off and sets GPIO to standard values
def cleanup():
    stop()
    stopServos()
    GPIO.cleanup()

# version(). Returns 2. Invalid until after init() has been called
def version():
    return 2 #(version 1 is Pirocon, version 2 is RoboHAT)

# End of General Functions
#======================================================================


#======================================================================
# Motor Functions
#
# stop(): Stops both motors
def stop():
    pinLeft_Plus.ChangeDutyCycle(0)
    pinLeft_Minus.ChangeDutyCycle(0)

    pinRight_Plus.ChangeDutyCycle(0)
    pinRight_Minus.ChangeDutyCycle(0)
    
# forward(speed): Sets both motors to move forward at speed. 0 <= speed <= 100
def forward(speed):
    pinLeft_Plus.ChangeDutyCycle(speed)
    pinLeft_Minus.ChangeDutyCycle(0)

    pinRight_Plus.ChangeDutyCycle(speed)
    pinRight_Minus.ChangeDutyCycle(0)

    pinLeft_Plus.ChangeFrequency(speed + 5)
    pinRight_Plus.ChangeFrequency(speed + 5)
    
# reverse(speed): Sets both motors to reverse at speed. 0 <= speed <= 100
def reverse(speed):
    pinLeft_Plus.ChangeDutyCycle(0)
    pinLeft_Minus.ChangeDutyCycle(speed)

    pinRight_Plus.ChangeDutyCycle(0)
    pinRight_Minus.ChangeDutyCycle(speed)

    pinLeft_Minus.ChangeFrequency(speed + 5)
    pinRight_Minus.ChangeFrequency(speed + 5)

# spinLeft(speed): Sets motors to turn opposite directions at speed. 0 <= speed <= 100
def spinLeft(speed):
    pinLeft_Plus.ChangeDutyCycle(0)
    pinLeft_Minus.ChangeDutyCycle(speed)

    pinRight_Plus.ChangeDutyCycle(speed)
    pinRight_Minus.ChangeDutyCycle(0)

    pinLeft_Minus.ChangeFrequency(speed + 5)
    pinRight_Plus.ChangeFrequency(speed + 5)
    
# spinRight(speed): Sets motors to turn opposite directions at speed. 0 <= speed <= 100
def spinRight(speed):
    pinLeft_Plus.ChangeDutyCycle(speed)
    pinLeft_Minus.ChangeDutyCycle(0)

    pinRight_Plus.ChangeDutyCycle(0)
    pinRight_Minus.ChangeDutyCycle(speed)

    pinLeft_Plus.ChangeFrequency(speed + 5)
    pinRight_Minus.ChangeFrequency(speed + 5)
    
# turnForward(leftSpeed, rightSpeed): Moves forwards in an arc by setting different speeds. 0 <= leftSpeed,rightSpeed <= 100
def turnForward(leftSpeed, rightSpeed):
    pinLeft_Plus.ChangeDutyCycle(leftSpeed)
    pinLeft_Minus.ChangeDutyCycle(0)

    pinRight_Plus.ChangeDutyCycle(rightSpeed)
    pinRight_Minus.ChangeDutyCycle(0)
    
    pinLeft_Plus.ChangeFrequency(leftSpeed + 5)
    pinRight_Plus.ChangeFrequency(rightSpeed + 5)
    
# turnReverse(leftSpeed, rightSpeed): Moves backwards in an arc by setting different speeds. 0 <= leftSpeed,rightSpeed <= 100
def turnReverse(leftSpeed, rightSpeed):
    pinLeft_Plus.ChangeDutyCycle(0)
    pinLeft_Minus.ChangeDutyCycle(leftSpeed)
    
    pinRight_Plus.ChangeDutyCycle(0)
    pinRight_Minus.ChangeDutyCycle(rightSpeed)
    
    pinLeft_Minus.ChangeFrequency(leftSpeed + 5)
    pinRight_Minus.ChangeFrequency(rightSpeed + 5)

# jwc for individual motor control
#
def motorLeft_Plus_Fn(speed_In):
    pinLeft_Plus.ChangeDutyCycle(speed_In)
    pinLeft_Minus.ChangeDutyCycle(0)
        
    pinLeft_Plus.ChangeFrequency(speed_In + 5)

def motorLeft_Minus_Fn(speed_In):
    pinLeft_Plus.ChangeDutyCycle(0)
    pinLeft_Minus.ChangeDutyCycle(speed_In)
        
    pinLeft_Minus.ChangeFrequency(speed_In + 5)


def motorRight_Plus_Fn(speed_In):
    pinRight_Plus.ChangeDutyCycle(speed_In)
    pinRight_Minus.ChangeDutyCycle(0)
        
    pinRight_Plus.ChangeFrequency(speed_In + 5)

def motorRight_Minus_Fn(speed_In):
    pinRight_Plus.ChangeDutyCycle(0)
    pinRight_Minus.ChangeDutyCycle(speed_In)
        
    pinRight_Minus.ChangeFrequency(speed_In + 5)



# End of Motor Functions
#======================================================================


#======================================================================
# IR Sensor Functions
#
# irLeft(): Returns state of Left IR Obstacle sensor
def irLeft():
    if GPIO.input(irFL)==0:
        return True
    else:
        return False
    
# irRight(): Returns state of Right IR Obstacle sensor
def irRight():
    if GPIO.input(irFR)==0:
        return True
    else:
        return False
    
# irAll(): Returns true if any of the Obstacle sensors are triggered
def irAll():
    if GPIO.input(irFL)==0 or GPIO.input(irFR)==0:
        return True
    else:
        return False
    
# irLeftLine(): Returns state of Left IR Line sensor
def irLeftLine():
    if GPIO.input(lineLeft)==0:
        return True
    else:
        return False
    
# irRightLine(): Returns state of Right IR Line sensor
def irRightLine():
    if GPIO.input(lineRight)==0:
        return True
    else:
        return False
    
# End of IR Sensor Functions
#======================================================================


#======================================================================
# UltraSonic Functions
#
# getDistance(). Returns the distance in cm to the nearest reflecting object. 0 == no object
def getDistance():
    GPIO.setup(sonar, GPIO.OUT)
    # Send 10us pulse to trigger
    GPIO.output(sonar, True)
    time.sleep(0.00001)
    GPIO.output(sonar, False)
    start = time.time()
    count=time.time()
    GPIO.setup(sonar,GPIO.IN)
    while GPIO.input(sonar)==0 and time.time()-count<0.1:
        start = time.time()
    count=time.time()
    stop=count
    while GPIO.input(sonar)==1 and time.time()-count<0.1:
        stop = time.time()
    # Calculate pulse length
    elapsed = stop-start
    # Distance pulse travelled in that time is time
    # multiplied by the speed of sound (cm/s)
    distance = elapsed * 34000
    # That was the distance there and back so halve the value
    distance = distance / 2
    return distance

# End of UltraSonic Functions    
#======================================================================

#======================================================================
# Servo Functions
# Pirocon/Microcon/RoboHAT use ServoD to control servos

def setServo(Servo, Degrees):
    global ServosActive
    #print "ServosActive:", ServosActive
    #print "Setting servo"
    if ServosActive == False:
        startServos()
    pinServod (Servo, Degrees) # for now, simply pass on the input values

def stopServos():
    #print "Stopping servo"
    stopServod()
    
def startServos():
    #print "Starting servod as CPU =", CPU
    startServod()
    
def startServod():
    global ServosActive
    #print "Starting servod. ServosActive:", ServosActive
    SCRIPTPATH = os.path.split(os.path.realpath(__file__))[0]
    #os.system("sudo pkill -f servod")
    # jwc 
    #
    ##jwc o initString = "sudo " + SCRIPTPATH +'/servod --pcm --idle-timeout=20000 --p1pins="18,22" > /dev/null'
    ##jwc oinitString = "sudo " + SCRIPTPATH +'/servod --pcm --idle-timeout=20000 --p1pins="24,25" > /dev/null'
    initString = "sudo " + SCRIPTPATH +'/servod --pcm --idle-timeout=20000 --p1pins="18,24" > /dev/null'
    
    os.system(initString)
    #print initString
    ServosActive = True

def pinServod(pin, degrees):
    #print pin, degrees
    pinString = "echo " + str(pin) + "=" + str(50+ ((90 - degrees) * 200 / 180)) + " > /dev/servoblaster"
    #print pinString
    os.system(pinString)
    
def stopServod():
    global ServosActive
    os.system("sudo pkill -f servod")
    ServosActive = False

