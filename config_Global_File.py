#!/usr/bin/env python3
##jwc o #!/usr/bin/env python2

# jwc from: ~/01-Jwc/03t-StsPilot-MarkOrion/STS-PiLot

# -*- coding: utf-8 -*-
"""
Created on Wed Mar 15 09:44:23 2017

@author: Mark Dammer

This file contains basic configuration and all former global variables
"""
# Configuration
width = 320 # Video width requested from camera
height = 240 # Video height requested from camera
video_src = 0 # OpenCV video source for camera_cv.py
pi_hflip = True # Flip Picamera image horizontally
pi_vflip = True # Flip Picamera image vertically
cv_hflip = False # Flip OpenCV camera image horizontally
cv_vflip = False # Flip OpenCV camera image vertically

# Global values
camera_detected = True
camera_active = True
camera = None
stream = None
video_status = True
video_fps = 0
brakes = False
chocks = False
blue = False
yellow = False
green = False
_watchdog_Server_Control_C_Inactive_Bool = True
_watchdog_EmergencyStop_Inactive_Bool = True
##jwc o _watchdog_Cycles_SinceLastConnect_Now = 20  ## 
_watchdog_Cycles_SinceLastConnect_MAX = 10
_watchdog_Cycles_SinceLastConnect_Now = _watchdog_Cycles_SinceLastConnect_MAX  ## default to disconnected ever since
left_motor = 0
right_motor = 0
# jwc 
# 
left_motor_trim = 0
right_motor_trim = 0
heartbeat_freq = 0

# default to 90 degrees
#
servo_01_Pan_Degrees = 90
servo_02_Tilt_Degrees = 90
servo_03_Degrees = 90
servo_04_Degrees = 90
temp = 0

# * BatteryUps: GeekPi/52pi.com: EP-0118
# https://wiki.52pi.com/index.php/UPS_(With_RTC_%26_Coulometer)_For_Raspberry_Pi_SKU:_EP-0118
#
_batteryUps_Input_V = 0
_batteryUps_Output_V = 0
_batteryUps_Battery_V = 0
_batteryUps_Temp_C = 0
_batteryUps_Temp_F = 0

# * Timer
_timer_Mission_Start_Sec = 0
_timer_Mission_Now_Sec = 0
_timer_Mission_Countdown_Sec = 0
_timer_Mission_Expired_Bool = True
_timer_Mission_Duration_MAX_SEC = 100
_timer_Mission_Reserves_Sec_Int = 0
_timer_Mission_Reserves_SEC_MAX_INT = 300
_timer_Mission_Recharge_Sec_Int = 0
_timer_Mission_Recharge_THRESHOLD_DEC = 0.10
_timer_Mission_Recharge_Timestamp_Int = 0

_trigger_Client_Req_01_Bool = False
_trigger_Client_Req_02_Bool = False
_trigger_Client_Req_03_Bool = False

_scanner_Client_AvgScore_Dec = 0.0

_trigger_Client_01_AvgScorePerTrigger_Dec = 0.0
_trigger_Client_02_AvgScorePerTrigger_Dec = 0.0
_trigger_Client_03_AvgScorePerTrigger_Dec = 0.0

_debug_Print_On = False
# Useful to attempt reduce Cpu/Net Lag by Setting to Falserm 
_camAi_On = True