#!/usr/bin/python

""" 

//////////////////////////////////////////////////////////////////////////////
OpenVNAVI (Open Vibrotactile Navigation Aid for the Visually Impaired)
Author: David Antón Sánchez
License: GPLv3
github.com/davidanton
//////////////////////////////////////////////////////////////////////////////

WORK IN PROGRESS 
BETA(TM)

THINGS TO IMPLEMENT:

- Crop image to delete therow of black pixels that appears on the left
- Modes: 
1: Gain, increase/decrease.
2: Threshold, values for far-away objects are neglected.
- Image stabilization
- Optimize for more FPS
- "frame16.astype(np.uint8)" doesn't work for some reason, it would have been 
easier to use that and map from 8-bit to 12-bit without the need for setting a
maxPWM. I used frame16.astype(int) and set maxPWM.

NOTES FOR DOCUMENTATION: 

Installation: 
OpenNI1 + PyOpenNI -> PyOpenNI doesn't compile
OpenNI2 + primesense Python bindings -> I don't remember error
OpenNI1 + OpenCV2 -> It works
	Installed OpenNI Unstable 1.5.4.0 but the other versions seem to work also,
	try with a stable version. 
	OpenNI built following readme
	OpenCV compiled from source -D WITH_OPENNI=YES 

"""

__author__ = "David Anton Sanchez"
__license__ = "GPLv3"


# Imports
# ============================================================================

import cv2
import numpy as np
import sys
import os
import time
import RPi.GPIO as GPIO
from PWM_driver import PWM


# Constants
# ============================================================================

""" Each device and channel is identified by an integer. The C++ 
version of OpenCV defines some constants for the identifiers of 
certain devices and channels. However, these constants are not 
defined in the Python version and the assignment needs to be 
performed manually. """

# Devices.
CV_CAP_OPENNI = 900 # OpenNI (for Microsoft Kinect)
CV_CAP_OPENNI_ASUS = 910 # OpenNI (for Asus Xtion)
# Channels of an OpenNI-compatible depth generator.
CV_CAP_OPENNI_DEPTH_MAP = 0 # Depth values in mm (CV_16UC1)
CV_CAP_OPENNI_POINT_CLOUD_MAP = 1 # XYZ in meters (CV_32FC3)
CV_CAP_OPENNI_DISPARITY_MAP = 2 # Disparity in pixels (CV_8UC1)
CV_CAP_OPENNI_DISPARITY_MAP_32F = 3 # Disparity in pixels (CV_32FC1)
CV_CAP_OPENNI_VALID_DEPTH_MASK = 4 # CV_8UC1
# Channels of an OpenNI-compatible RGB image generator.
CV_CAP_OPENNI_BGR_IMAGE = 5
CV_CAP_OPENNI_GRAY_IMAGE = 6


# Definitions
# ============================================================================

# Debugging and test functions:

def depthTest():

	""" Grabs frames from different channels, multiplies the output 
	by different constant gain values and saves the PNG frames 
	for visual inspection of the range of depth values of a scene. 
	The folder named "test_frames" must be created. """

	abc = (["a","b","c","d","e","f","g","h","i","j","k","l","m",
		  "n","o","p","q","r","s","t","u","v","w","x","y","z"])
	values = 7
	testGain = 5
	maxChannel = 4

	for x in range(0, values):
		for y in range(0, maxChannel):
			capture.grab()
			print "Retrieving channel " + str(y) + abc[x]
			success, rawFrame = capture.retrieve(channel = y)
			frame640y = testGain * int(x) * rawFrame
			cv2.imwrite("test_frames/" + str(y) + abc[x] + ".png", frame640y)

def sweepTest():

	""" Sweeps up and down for all motors. """

	minPWM = 300  
	maxPWM = 3000
	step = 100  

	for i in range(minPWM,maxPWM,step):
		for j in range(0,8):
			for k in range(0,16):
				IC[j].setPWM(k, 0, i)
				time.sleep(0.001)
				print "IC: " + str(j) + " motor: " + str(k) + " PWM: " + str(i)

	for i in range(maxPWM,minPWM,-step):
		for j in range(0,8):
			for k in range(0,16):
				IC[j].setPWM(k, 0, i)
				time.sleep(0.001)
				print "IC: " + str(j) + " motor: " + str(k) + " PWM: " + str(i)

def strobeTest(row):

	""" Strobes maximum and minimum values for a given row. """

	minPWM = 300  
	maxPWM = 3000 

	IC[row].setAllPWM(0, minPWM)
	time.sleep (1)
	IC[row].setAllPWM(0, maxPWM)
	time.sleep (1)


# Other functions:

def startUp():

	""" Ramps from low to medium vibration values for a smooth
	transition. """

	minPWM = 300  
	maxPWM = 1000
	step = 10 

	for i in range(minPWM,maxPWM,step):
		for j in range(0,8):
			IC[j].setAllPWM(0, i)

def beep(number, t):

	""" Simple buzzer beep. """

	GPIO.setwarnings(False)
	GPIO.setup(12, GPIO.OUT)
	for i in range (0, number):
		buzzer = GPIO.PWM(12, 5000)
		time.sleep(t)
		buzzer.start(92)
		time.sleep(t)
		buzzer.stop()

def pause():

	""" Pause function. """

	for i in range(0, 8):
		IC[i].setAllPWM(0, 0)
	print "System paused by the user."
	time.sleep(1)
	beep(2, 0.2)
	print "System ready, press switch to continue..."
	flag = 1
	GPIO.wait_for_edge(18, GPIO.RISING)
	startUp()

def getFrame():

	""" Gets the frame from the sensor and stores it it. 
	Returns 16x8 numpy array. """

	capture.grab()
	success, rawFrame = capture.retrieve(channel = channel)
	frame640 = gain * rawFrame
	frame16 = cv2.resize(frame640, (16, 8)) 
	frame16 = frame16.astype(int)
	# For debugging 
	# cv2.imwrite("frame640.png", frame640) 
	# cv2.imwrite("frame16.png", frame16)
	# print frame16
	return frame16

def setVibration():

	""" Sets PWM values to each vibration motor unit. """
 	
	mappingFunction = 8 # Mapping of grayscale values to PWM values.
	PWM16 = mappingFunction * getFrame()
	maxPWM = 3060
	for row in range(0,8):
		for col in range (0,16):
			if PWM16[row,col] > maxPWM:
				PWM16[row,col] = maxPWM
			elif PWM16[row,col] == 0:
				PWM16[row,col] = maxPWM
			else:
				IC[row].setPWM(col,0,(PWM16[row,col]))
				# print (PWM16[row,col]),
		# print "\n"


# Main function
# ============================================================================

# Initialization of PWM drivers. Use PWM(0x40, debug=True) for debugging. 
IC = []
freq = 1000 #PWM frequency [Hz].
for i in range(0,8):
	IC.append(PWM(0x40+i))
	IC[i].setPWMFreq(freq)

# Initialization of the sensor. 
sensor = CV_CAP_OPENNI_ASUS
channel = 3
gain = 5
capture = cv2.VideoCapture(sensor)
capture.open(sensor) 
while not capture.isOpened():
	print "Couldn't open sensor. Is it connected?"	
	time.sleep(100)
print "Sensor opened successfully"

# GPIO initialization.
GPIO.setmode(GPIO.BCM)
GPIO.setup(18, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
sw1 = GPIO.input(18) # Input NO switch.
flag = 0 # Flag for start/stop button.

# Waits for sw1 to be pressed.
print "System ready, press switch to continue..."
beep(1, 0.2)
GPIO.wait_for_edge(18, GPIO.RISING)

startUp()

def main():

    try:
		while True:

			if (GPIO.input(18) == False) or (GPIO.input(18) == True and flag == 1):
				flag = 0 
				tick = time.clock() 

				# depthTest()

				# sweepTest()

				# strobeTest(0)

				# getFrame()

				setVibration()

				# Calculates loop runtime.
				tock = time.clock()
				runtime = tock - tick
				print "Loop runtime: " + str(runtime) + "s"
				print "FPS: " + str(int(1/runtime))

			else:
				pause()

    except KeyboardInterrupt:
    	GPIO.cleanup
    	for i in range(0, 8):
    		IC[i].setAllPWM(0, 0)
        print "Shutdown requested. \nExiting..."

    finally:  
    	print "Done"


if __name__ == "__main__":
    main()
