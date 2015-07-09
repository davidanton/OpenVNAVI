#!/usr/bin/python

""" 

WORK IN PROGRESS 
ALPHA VERSION

Implement a start/stop with a button and a buzzer
Implement a slow motor sweep to mid values to get the user used to the feeling (with setAllPWM)
Implement modes: 1: Max/min: (span shift) const*PWM16 (fix max, moving min); 2: Sensitivity, values
for far-away objects are neglected.
Make one motor vibrate a number of times to indicate the mode when the button is pressed.
Implement image stabilization


NOTES: 

Installation: 
OpenNI1 + PyOpenNI -> PyOpenNI doesn't compile
OpenNI2 + primesense Python bindings -> I don't remember error
OpenNI1 + OpenCV2 -> It works
	Installed OpenNI Unstable 1.5.4.0 but the other versions seem to work also,
	try with a stable version. 
	OpenNI built following txt doc guide
	OpenCV compiled from source -D WITH_OPENNI=YES

To turn motors completely off (ojo, 4096!): setPWM(pin,4096,0)
Decide wether to install libraries on /usr/local/lib/python2.7/dist-packages, 
or just put them on the same folder and import them."""

__author__ = 'David Anton Sanchez'
__license__ = 'GPLv3'


# Imports
# ============================================================================

import cv2
import numpy
import sys
import os
import time
from PWM_driver import PWM


# Constants
# ============================================================================

""" Each device and channel is identified by an integer. The C++ 
version of OpenCV defines some constants for the identifiers of 
certain devices and channels. However, these constants are not 
defined in the Python version and it needs to be performed manually. """

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

# Debugging and testing functions:

def depthTest():

	""" Grabs frames from different channels, multiplies the output 
	by different constant gain values and saves the PNG frames 
	for visual inspection of the range of depth values of a scene. 
	The folder named "test_frames" must be created """

	abc = (["a","b","c","d","e","f","g","h","i","j","k","l","m",
		  "n","o","p","q","r","s","t","u","v","w","x","y","z"])
	values = 7
	gain = 5
	maxChannel = 4

	for x in range(0, values):
		for y in range(0, maxChannel):
			capture.grab()
			print "Retrieving channel " + str(y) + abc[x]
			success, rawFrame = capture.retrieve(channel = y)
			frame640y = gain * int(x) * rawFrame
			cv2.imwrite("test_frames/" + str(y) + abc[x] + ".png", frame640y)

def sweepTest():

	""" Sweep up for all motors """

	minPWM = 300  
	maxPWM = 2500  
	for i in range(minPWM,maxPWM,20):
		for j in range(0,7):
			for k in range(0,16):
		  		IC[j].setPWM(k, 0, i)
		  		time.sleep(0.001)
		  		print "IC: " + str(j) + " motor: " + str(k) + " PWM: " + str(i)


# Other functions:

def getFrame():

	""" Gets the frame from the sensor and stores it it. 
	Returns a x-bit 16x8 array """

	capture.grab()
	success, rawFrame = capture.retrieve(channel = 0)
	frame640 = gain * rawFrame
	# Resizes the image to 16x8 and stores the grayscale values in an array. 
	# (change 8 to 12 to maintain aspect ratio, but change the limits  of the loop as well!) 
	frame16 = cv2.resize(frame640, (16, 8)) 
	cv2.imwrite('frame640.png', frame640)
	cv2.imwrite('frame16.png', frame16)
	print frame16
	return frame16

def setVibration():

	""" Sets PWM values to each vibration motor unit. """
	PWM16 = mappingFunction * frame16
	for row in range(0,8):
		for col in range (0,16):
			IC[row].setPWM(col,0,(PWM16[row,col]))
			print (frame16[row,col]*16+15),
		print "\n"


# Main function
# ============================================================================

# Initialization of PWM drivers. Use PWM(0x40, debug=True) for debugging. 
IC = []
freq = 490 #PWM frequency [Hz]
for i in range(0,8):
	IC.append(PWM(0x40+i))
	IC[i].setPWMFreq(freq)

# Initialization of the sensor
sensor = CV_CAP_OPENNI_ASUS
channel = 0
gain = 15
capture = cv2.VideoCapture(sensor)
capture.open(sensor) 

if not capture.isOpened():
	print "Couldn't open device. Is it connected?"
else:
	print "Device opened successfully"

# Mapping of 8-bit grayscale values to 12-bit PWM values. 
mappingFunction = 1

def main():

    try:

		while True:
			# For loop runtime calculation
			tick = time.clock() 
			
			# depthTest()
			
			sweepTest()

			# getFrame()

			# setVibration()

			# Calculates loop runtime
			tock = time.clock()
			runtime = tock - tick
			print "Loop runtime: " + str(runtime) + "s"
			print "FPS: " + str(1/runtime)
  
    except KeyboardInterrupt:
    	for i in range(0,7):
    		IC[i].setAllPWM(0, 0)
        print "Shutdown requested...exiting"
    except Exception:
        traceback.print_exc(file=sys.stdout)
    sys.exit(0)

if __name__ == "__main__":
    main()







# Things to try:
# depth = depth.astype(np.uint8)
# If Asus image is not grayscale, convert by using cv2.CV_LOAD_IMAGE_GRAYSCALE
# Reads the 640x480 depth frame. OpenCV reads BGR by default, here it's loaded as grayscale.
# frame640 = cv2.imread('frame641.png', cv2.CV_LOAD_IMAGE_UNCHANGED)
