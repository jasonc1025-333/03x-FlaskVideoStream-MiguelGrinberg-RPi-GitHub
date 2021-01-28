#!/usr/bin/env python
# -*- coding: utf-8 -*-

######################################################################
# (C) ALCHEMY POWER INC 2016,2017 etc. - ALL RIGHT RESERVED.
# CODE SUBJECT TO CHANGE AT ANY TIME.
# YOU CAN COPY/DISTRIBUTE THE CODE AS NEEDED AS LONG AS YOU MAINTAIN
# THE HEADERS (THIS PORTION) OF THE TEXT IN THE FILE.
######################################################################
# Last update 06/10/2019
# Compensated for V drop in R-C filter
# Adjusted temp measurement to reflect R.
# Update 11/8/2019
# Added two different lines for measuring temperature on PiZ-UpTime and Pi-UpTime as 
# they use different NTC's.
#
# PLEASE READ NOTES BELOW. THERE IS NO README FILE.


# NOTE - temperature measurement is + or - 2 C
#
# Please change the temperature calculation appropriately depending on produyct you are using.
#
# Search for temperature_ViaVolts an you will find it documented.
#
# If you are running this script in the background, you may not need to capture all the data. You can comment out the print statements.
#
# Save a copy of this code if you comment out the print statements as you may need to run this interactively to see operating conditions. 
#
#
# Change the operating range alerts by changing values for temp_Min_DEC_GLOBAL and temp_Max_DEC_GLOBAL
# Operating range is assumed to be 5 C to 50 C. Note - this is the temp on the UpTime 
# board and has nothing to do with battery temperature. 
# 
#
# This script will print the instantenous voltages and temperature reading. 
# Disable (comment out) print statements to run the code in the background. 
# Enable print to run the code interactively.
#

import time
##jwc o n upgrade for python3\ import smbus
## import smbus2 as smbus
import sys
assert ('linux' in sys.platform), "This code runs on Linux only."
import os
import signal
import subprocess

##jwc o from smbus import SMBus
from smbus2 import SMBus

from sys import exit

# for older PI's (version 1) use smb_I2c_Cl_Ob = SMBus(0) in statement below.
smb_I2c_Cl_Ob = SMBus(1)
# One of the 3 possible addresses of the unit.
address_I2c_Hex_Global = 0x48
# address_I2c_Hex_Global = 0x49
# address_I2c_Hex_Global = 0x4B
#
# address_I2c_Hex_Global is the I2C address of the ADC chip.
# Use i2cdetect -y 1 to find the address. Use "y 0" for older Pi's.
#
# Depending on the jumper settings the address will change. Only 3 addresses are allowed. 
# Please make sure you use the proper address, else the script will give an error. 
#

#
# Chip used is TI TLA2024 - 4 channel, 12 bit ADC.
# See Page 18 and then Page 17 of data sheet for how to configure the bits.
# Bits set for specific channel, single shot conversion, PGA of 6.144V
# Error  or sensitivity is 3mV for settings below.
channel_0_BIT_GLOBAL        =     0b11000001   # Measure V-in
channel_1_BIT_GLOBAL        =     0b11010001   # Measure V-out
channel_2_BIT_GLOBAL        =     0b11100001   # Measure V-battery
channel_3_BIT_GLOBAL        =     0b11110001   # Measure V across NTC, to measure Temperature.
# Use these channel settings for volts_Ref_DEC_GLOBAL = 2.048V
# channel_0_BIT_GLOBAL        =     0b11000101   # Measure V-in
# channel_1_BIT_GLOBAL        =     0b11010101   # Measure V-out
# channel_2_BIT_GLOBAL        =     0b11100101   # Measure V-battery
# channel_3_BIT_GLOBAL        =     0b11110101   # Measure V across NTC, to measure Temperature.
#
# Data Rate set to default of 128 SPS
#
#
#####################################################################################################################
# Determine the maximum voltage - see notes above or page 18 of datasheet.
#####################################################################################################################
volts_Ref_DEC_GLOBAL = 6.144 # This is the max Vref - allows us to measure volts_In up to volts_In + 0.3V
# volts_Ref_DEC_GLOBAL = 2.048 # This is the max Vref with second set of channel settings
# volts_Ref_DEC_GLOBAL is determined by the bit setting specified in channel# above. See Table 7, PGA[2,0]
#####################################################################################################################


max_Reading_Mask_DEC_GLOBAL = 2047.0 # 2^11 - 1 = 12 bits of information, with MSB set to zero. See page 15 of data sheet.

# Now we determine the operating parameters.
# numberOfBytesToRead_HEX_GLOBAL = number of bytes to read. 
# sleepTime_BetweenCycles_INT_GLOBAL (German for time) - tells how frequently you want the readings to be read from the ADC. Define the
# time to sleep between the readings.
# sleepTime_BetweenReadings_DEC_GLOBAL (Spanish - noun - for time) shows how frequently each channel is read in over the I2C bus. Best to use
# timepo between each successive readings.
#
# All the timeouts and other operational variables.
numberOfBytesToRead_HEX_GLOBAL = 0x02 # number of bytes to read in the block. Need for Debug statements below.
sleepTime_BetweenCycles_INT_GLOBAL = 2     # number of seconds to sleep between each measurement group. This will read variables every 20 seconds.
sleepTime_BetweenReadings_DEC_GLOBAL = 0.1 # number of seconds to sleep between each channel reading.
# sleepTime_BetweenReadings_DEC_GLOBAL = 1/SPS (default is 128) + 1 ms - so sleepTime_BetweenReadings_DEC_GLOBAL can be as low as 0.002 seconds. We deliberately make it
# higher here for readings to settle down. Fast enough for us. In fact, in many cases this may not even be needed.
#####################################################################################################################
#####################################################################################################################
# Battery V
#=========================================================================================================
volts_Battery_Min_DEC_GLOBAL = 3.1 # Minimum V of battery at which time shutdown is triggered.
# Change that if you want the shutdown to initiate sooner. Note at 2.5V all electronics
# will be shut down.
#=========================================================================================================
# Min Voltage
#=========================================================================================================
volts_Input_Min_DEC_GLOBAL = 3.8 # Minimum input Voltage. Below this voltage level, it is not recommended to operate the
			   # Pi. Called a brown-out condition. We display result as 0.
#=========================================================================================================
# Temperature
#=========================================================================================================
# Temp min/max is the min and max temperature for which battery can charge.
# This can be changed to suite your needs.
# Note the hardware does not allow charging for temp_Min_DEC_GLOBAL of 0C
# and a maximum of 50C.

temp_Min_DEC_GLOBAL = 5.0 # Min temperature at which battery charging enclose should be warmed up.
temp_Max_DEC_GLOBAL = 60.0 # Max temperature after which battery charging should be discontinued.
				# Best to provide cooling e.g. a fan.
# This range is lower/higher by about 10 degrees to allow corrective action.

#=========================================================================================================


# This is a subroutine which is called from the main routine. 
# The variables passed are:
# adc_Address_In - I2C address for the device. 
# adc_channel_In - the analog channel you want to read in.
#####################################################################################################################
#####################################################################################################################

def get_Reading_Fn(adc_Address_In,adc_channel_In):
	global smb_I2c_Cl_Ob, sleepTime_BetweenReadings_DEC_GLOBAL
	
#Debug
#	print "ADC Address and channel is ", adc_Address_In, adc_channel_In, "Sleeping for ", sleepTime_BetweenReadings_DEC_GLOBAL, " seconds."

# Reset the registers (address and data) and then read the data.
	smb_I2c_Cl_Ob.write_i2c_block_data(adc_Address_In, 0x01, [0x85, 0x83]) #Reset config register
	smb_I2c_Cl_Ob.write_i2c_block_data(adc_Address_In, 0x00, [0x00, 0x00]) #Reset data register
# Wait till the reading stabilize.
	time.sleep(sleepTime_BetweenReadings_DEC_GLOBAL) # Wait for conversion to finish
# Trigger the ADC for a one-shot reading on the channel.
	smb_I2c_Cl_Ob.write_i2c_block_data(adc_Address_In, 0x01, [adc_channel_In, 0x43]) # Initialize channel we want to read.
	time.sleep(sleepTime_BetweenReadings_DEC_GLOBAL) # Wait for conversion to finish
#
# Debug
# Print the Config Register
#	readread = smb_I2c_Cl_Ob.read_i2c_block_data(adc_Address_In, 0x01, numberOfBytesToRead_HEX_GLOBAL)
#	print "Config Reg is ", readread[1], readread[0]

# Read the data register.
	reading  = smb_I2c_Cl_Ob.read_word_data(adc_Address_In, 0) # Read data register
#Debug - print the data read
#	print "Data register read as 16 but word is ", reading

# Do the proper bit movements. Refer to data sheet for how the bits are read in.
	valor = ((((reading) & 0xFF) <<8) | ((int(reading) & 0xFFF0)>>8))
	valor = valor >> 4 # 4 LSB bits are ignored.

# Debug print the bit movement value...
#	print("Valor is 0x%x" % valor)

	volts_Out = valor/max_Reading_Mask_DEC_GLOBAL*volts_Ref_DEC_GLOBAL

	return volts_Out

# End of sub routine
#####################################################################################################################
#####################################################################################################################
# Define the keyboard interrupt routine.
def keyboardInterruptHandler_Fn(signal_In, frame_In):
	print
	print("KeyboardInterrupt (ID: {}) detected. Cleaning up...".format(signal_In))
	sys.stdout.flush()	
	exit(0)
#####################################################################################################################
#####################################################################################################################
##jwc o # Start watching for Keyboard interrupt
##jwc o signal.signal(signal.SIGINT, keyboardInterruptHandler_Fn)

# Main routine. 

ch_0_Multiplier_INT_GLOBAL = 1 # Multiplier is used to offset any values and for calibration adjustments. volts_In
ch_1_Multiplier_INT_GLOBAL = 1 # Multiplier for Channel 1 - volts_Out
ch_2_Multiplier_INT_GLOBAL = 1 # Multiplier for Channel 2 - volts_Battery
ch_3_Multiplier_INT_GLOBAL = 1 # Multiplier for Channel 3 - V for Temperature at the NTC (thermistor).

##jwc o replace by PiUpTimeUpsef batteryUps_Read_Fn(config_In):
##jwc o replace by PiUpTimeUps   global batteryUps_ClObj_Global
##jwc o replace by PiUpTimeUps
##jwc o replace by PiUpTimeUps   config_In.batteryUps_Volts_Input_V = batteryUps_ClObj_Global.voltage()
##jwc o replace by PiUpTimeUps   ##jwc y print("*** DEBUG: batteryUps_Volts_Input_V: %.3f V" % config_In.batteryUps_Volts_Input_V)
##jwc o replace by PiUpTimeUps   print(f"*** DEBUG: batteryUps_Volts_Input_V: {config_In.batteryUps_Volts_Input_V:.2f} V", end='')
##jwc o replace by PiUpTimeUps   try:
##jwc o replace by PiUpTimeUps       config_In.batteryUps_Volts_Output_V = batteryUps_ClObj_Global.shunt_voltage()
##jwc o replace by PiUpTimeUps       ##jwc y print("*** DEBUG: batteryUps_Volts_Output_V: %.3f mV" % config_In.batteryUps_Volts_Output_V)
##jwc o replace by PiUpTimeUps       print(f" // batteryUps_Volts_Output_V: {config_In.batteryUps_Volts_Output_V:.2f} mV", end='')
##jwc o replace by PiUpTimeUps
##jwc o replace by PiUpTimeUps       config_In.batteryUps_Temp_C = batteryUps_ClObj_Global.current()
##jwc o replace by PiUpTimeUps       ##jwc y print("*** DEBUG: batteryUps_Temp_C: %.3f mA" % config_In.batteryUps_Temp_C)
##jwc o replace by PiUpTimeUps       print(f" // batteryUps_Temp_C: {config_In.batteryUps_Temp_C:.2f} mA", end='')
##jwc o replace by PiUpTimeUps
##jwc o replace by PiUpTimeUps       config_In.batteryUps_Temp_F = batteryUps_ClObj_Global.power()
##jwc o replace by PiUpTimeUps       ##jwc y print("*** DEBUG: batteryUps_Temp_F: %.3f mW" % config_In.batteryUps_Temp_F)
##jwc o replace by PiUpTimeUps       print(f" // batteryUps_Temp_F: {config_In.batteryUps_Temp_F:.2f} mW)")
##jwc o replace by PiUpTimeUps   except DeviceRangeError as e:
##jwc o replace by PiUpTimeUps       print(e)


def get_VoltageAndTemp_Status_Fn(config_In):
	global ch_0_Multiplier_INT_GLOBAL, ch_1_Multiplier_INT_GLOBAL, ch_2_Multiplier_INT_GLOBAL, ch_3_Multiplier_INT_GLOBAL, address_I2c_Hex_Global
	global channel_0_BIT_GLOBAL, channel_1_BIT_GLOBAL, channel_2_BIT_GLOBAL, channel_3_BIT_GLOBAL
	# Main routine - shows an endless loop. If used with a cron script or a 
	# trigger via GPIO, an endless loop is not recommended.
	#
	#
	print ("*** DEBUG: Date & Time               volts_In   volts_Out  Batt-V  Board Temperature")
	##jwc o while (True):
	# Read Channel 0 - Input Voltage - max 5.5V.
	volts_In = ch_0_Multiplier_INT_GLOBAL*get_Reading_Fn(address_I2c_Hex_Global,channel_0_BIT_GLOBAL)
	config_In.batteryUps_Volts_Input_V = volts_In
	#	if (volts_In < volts_Input_Min_DEC_GLOBAL):
	#		volts_In = 0
	# Sleep between each reading. Sleep time is in the subroutine. Use this to slow down readings further. 
	# Add sleep as needed.
	#	time.sleep(sleepTime_BetweenReadings_DEC_GLOBAL)

	# Read Channel 1 - Battery V
	volts_Battery = ch_1_Multiplier_INT_GLOBAL*get_Reading_Fn(address_I2c_Hex_Global, channel_1_BIT_GLOBAL) + 0.2
	config_In.batteryUps_Volts_Battery_V = volts_Battery

	# Read Channel 2 - Output V
	volts_Out = ch_1_Multiplier_INT_GLOBAL*get_Reading_Fn(address_I2c_Hex_Global, channel_2_BIT_GLOBAL)
	config_In.batteryUps_Volts_Output_V = volts_Out

	# Read Channel 3 - Temperature.
	temperature_ViaVolts = ch_3_Multiplier_INT_GLOBAL*get_Reading_Fn(address_I2c_Hex_Global, channel_3_BIT_GLOBAL)

	# For Pi-Z-UpTime 2.0 use the value below.
	#	temperature_InCelcius_Dec = (4.0 - temperature_ViaVolts) / 0.0432 # Temperature in C calculated.
	# Use the below line for Pi-UpTime UPS 2.0
	temperature_InCelcius_Dec = (4.236 - temperature_ViaVolts) / 0.0408 # Temperature in C calculated.
	config_In.batteryUps_Temp_C = temperature_InCelcius_Dec

	# Line below computes Temperature in F from C
	temperature_InFahrenheit_Dec = temperature_InCelcius_Dec * 1.8 + 32.0  # Temperature in F
	config_In.batteryUps_Temp_F = temperature_InFahrenheit_Dec

	# Temperature is measured by measuring the V across the NTC. We assume a linear behavior in the use range.
	# According to Murata data sheet, the measurement of temperature is determined as below.
	#	R = R_ambient * exp (B) (1/T - 1/Tambient)
	#	R_ambient = 10K
	#	T_ambient = 25 C (need this in kelvin) = 298.15 K
	#	We can avoid complexity in calculation by assuming linearity.
	#   Doing some measurements and calculations
	#  Depending on Thermister in use, 
	#  Temperature = -0.00812 * Temp_V + 9.164
	#   Temperature = -0.0408 * Temp_V + 4.0
	#     where Temp_V is the Voltage measured by ADC
	#   Temperature is in degrees C.
	# Print values calculated so far.

	#	if(volts_In < volts_Input_Min_DEC_GLOBAL):
	#		print ("%s %5.2f %5.2f %5.2f    volts_In Failure - not charging" % (time.ctime(), volts_In, volts_Out, volts_Battery)) 
	#	else:
	#		print ("%s %5.2f %5.2f %5.2f %8.2fC %6.2fF" % (time.ctime(), volts_In, volts_Out, volts_Battery, temperature_InCelcius_Dec, temperature_InFahrenheit_Dec)) 
	##jwc O print ("%s %5.2f %5.2f %5.2f %8.2fC %6.2fF" % (time.ctime(), volts_In, volts_Out, volts_Battery, temperature_InCelcius_Dec, temperature_InFahrenheit_Dec)) 
	print ("*** DEBUG: %s %5.2f %5.2f %5.2f %8.2fC %6.2fF" % (time.ctime(), volts_In, volts_Out, volts_Battery, temperature_InCelcius_Dec, temperature_InFahrenheit_Dec)) 
	# Write the values read should there be an interrupt. Since output is to stdout, it needs to be flushed
	##
	#====================================================================================
	# Check to see if all operating conditions are OK. This code is a duplicate of
	# the shutdown code used with the cron script.
	#====================================================================================
	# If input V is low and battery V is low initiate the shutdown process.
	if ( volts_In < volts_Input_Min_DEC_GLOBAL ): # volts_In has failed or is a brownout
			if (volts_Battery < volts_Battery_Min_DEC_GLOBAL ): # Battery is low, time to shutdown.
					##jwc o print("Shutdown initiated at %s " % (time.ctime()))
					print("*** DEBUG: Vbattery < V_batt_min: Shutdown initiated at %s " % (time.ctime()))
					#
					# The command shuts down the pi in 2 minutes. Replace
					# 2 with word "now" (without the quotes) for immediate
					# shutdown. If you use the Pi-Zero-Uptime (the one which uses 14500 battery)
					# recommend using "shutdown -h now" instead of shutdown -h 2.
					# The subprocess method forks a process which can run in the background while this
					# program exits properly. os.system method continues to run in the program thread.
					#
					##jwc O print ("At %s, Vin = %4.2f, Vout = %4.2f, Vbattery = %4.2f, Temperature = %5.2fC %5.2fF" % (time.ctime(), Vin, Vout, Vbattery, TempC,TempF)) # Print the values see to initiate the shutdown.
					print ("*** DEBUG: Vbattery < V_batt_min: At %s, Vin = %4.2f, Vout = %4.2f, Vbattery = %4.2f, Temperature = %5.2fC %5.2fF" % (time.ctime(), Vin, Vout, Vbattery, TempC,TempF)) # Print the values see to initiate the shutdown.
					# print ("At %s, Vin = %4.2f, Vout = %4.2f, Vbattery = %4.2f" % (time.ctime(), Vin, Vout, Vbattery)) # Print the values see to initiate the shutdown.
					##jwc O sys.stdout.flush()
					##jwc O subprocess.call("shutdown -h 2 &", shell=True)

					#
					# Sleep for a second or so for the shutdown process to fork and then exit
					# Print the values out if needed - ok for debug.
					##jwc O time.sleep(2)
					# Flush any stdout messages before exiting..
					##jwc O exit() # Exit out of the code - no further print etc. is printed.
				#
				# Note the if statement falls out of the code if all is well.
				#
	# You can comment out the temperature monitoring if you desire. The hardware will ensure temperature is in the operating
	# range.
	# Lets monitor temperature on the board.
	if(volts_In > volts_Input_Min_DEC_GLOBAL): # volts_In is off - so temperature is not as critical for charging
		if (temperature_InCelcius_Dec < temp_Min_DEC_GLOBAL): # Temperature is too cold
				print ("Temperature is too cold for battery charging - at %s, Temperature is  %5.2f" % (time.ctime(), temperature_InCelcius_Dec))
	if(volts_In > volts_Input_Min_DEC_GLOBAL): # volts_In is off - so temperature is not as critical for charging
		if (temperature_InCelcius_Dec > temp_Max_DEC_GLOBAL): # Temperature is too hot
				print ("Temperature is too hot for battery charging -  at %s, Temperature is  %5.2f" % (time.ctime(), temperature_InCelcius_Dec))
	# You can modify the code to page you or send you an email if the temperature gets too hot. A print statement is in
	# in place as a place holder.


	#====================================================================================
	# End of check statements.
	#====================================================================================
	##jwc o sys.stdout.flush()
	##jwc o time.sleep(sleepTime_BetweenCycles_INT_GLOBAL)
	return 'ok'
