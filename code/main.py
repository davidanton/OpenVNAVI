#!/usr/bin/python
# -*- coding: utf-8 -*-

"""
//////////////////////////////////////////////////////////////////////////////
==============================================================================
OpenVNAVI (Open Vibrotactile Navigation Aid for the Visually Impaired)
Author: David Antón Sánchez
License: GPLv3
github.com/davidanton
==============================================================================
//////////////////////////////////////////////////////////////////////////////
"""

__author__ = "David Anton Sanchez"
__license__ = "GPLv3"


# ============================================================================
# Imports
# ============================================================================
import cv2
import numpy as np
import sys
import os
import time
import RPi.GPIO as GPIO
from PWM_driver import PWM
from flask import Flask, jsonify, render_template, request
import multiprocessing as mp


# ============================================================================
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


# ============================================================================
# Definitions
# ============================================================================
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
    for i in range(0, values):
        for j in range(0, maxChannel):
            capture.grab()
            print "Retrieving channel " + str(j) + abc[i]
            success, rawFrame = capture.retrieve(channel = j)
            frame640j = testGain * i * rawFrame
            cv2.imwrite("test_frames/" + str(j) + abc[i] + ".png", frame640j)

def sweepTest():

    """ Sweeps up and down for all motors. """

    minPWM = 300
    maxPWM = 3000
    step = 100
    for i in range(minPWM,maxPWM,step):
        for row in range(0,8):
            for col in range(0,16):
                IC[row].setPWM(col, 0, i)
                time.sleep(0.001)
                print ("IC: " + str(row) + " motor: " +
                      str(col) + " PWM: " + str(i))

    for i in range(maxPWM,minPWM,-step):
        for row in range(0,8):
            for col in range(0,16):
                IC[row].setPWM(col, 0, i)
                time.sleep(0.001)
                print ("IC: " + str(row) + " motor: " +
                      str(col) + " PWM: " + str(i))

def strobeTest(row):

    """ Strobes maximum and minimum values for a given row. """

    minPWM = 300
    maxPWM = 3000
    IC[row].setAllPWM(0, minPWM)
    time.sleep (1)
    IC[row].setAllPWM(0, maxPWM)
    time.sleep (1)

def fadeIn():

    """ Ramps from low to medium vibration values for a smooth
    transition. """

    minPWM = 300
    maxPWM = 1000
    step = 10
    for i in range(minPWM,maxPWM,step):
        for row in range(0,8):
            IC[row].setAllPWM(0, i)

def fadeOut():

    """ Ramps from medium to low vibration values for a smooth
    transition. """

    minPWM = 300
    maxPWM = 1000
    step = 10
    for i in range(maxPWM, minPWM, -step):
        for row in range(0,8):
            IC[row].setAllPWM(0, i)

def beep(number, t):

    """ Simple buzzer beep. Takes number of beeps and the duration. """

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

    fadeOut()
    for row in range(0, 8):
        IC[row].setAllPWM(0, 0)
    print "System paused by the user."
    time.sleep(0.5)
    beep(1, 0.2)
    print "System ready, press switch to continue..."
    flag = 1
    GPIO.wait_for_edge(18, GPIO.RISING)
    fadeIn()

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

def setVibration(frame = getFrame()):

    """ Sets PWM values to each vibration motor unit. """

    mappingFunction = 8 # Mapping of grayscale values to PWM values.
    PWM16 = mappingFunction * frame
    maxPWM = 3060
    for row in range(0,8):
        for col in range (0,16):
            if PWM16[row,col] > maxPWM:
                PWM16[row,col] = maxPWM
                IC[row].setPWM(col,0,(PWM16[row,col]))
            elif PWM16[row,col] == 0:
                PWM16[row,col] = maxPWM
                IC[row].setPWM(col,0,(PWM16[row,col]))
            else:
                IC[row].setPWM(col,0,(PWM16[row,col]))
                # print (PWM16[row,col]),
        # print "\n"


# ============================================================================
# Vibration renderer process
# ============================================================================

def rendererProcess(queue):

    """ Entry point for the renderer process. """

    try:
        # Initialization of PWM drivers. PWM(0x40, debug=True) for debugging.
        global IC
        IC = []
        freq = 490
        for i in range(0,8):
            IC.append(PWM(0x40+i))
            IC[i].setPWMFreq(freq)

        # Initialization of the sensor.
        global sensor
        sensor = CV_CAP_OPENNI_ASUS
        global channel
        channel = 3
        global gain
        gain = 5
        global capture
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
        fadeIn()

        while True:
            if ((GPIO.input(18) == False) or
                (GPIO.input(18) == True and flag == 1)):
                flag = 0
                tick = time.clock()

                # depthTest()
                # sweepTest()
                # strobeTest(0)
                # getFrame()
                if not queue.empty():
                    requestJson = queue.get()
                    mode = requestJson["mode"]

                    if mode == "web":
                        frame = np.array(requestJson["motors"])
                        setVibration(frame)
                    elif mode == "kinect":
                        setVibration()
                else:
                    setVibration()

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


# ============================================================================
# Web server process
# ============================================================================
webServer = Flask(__name__)
webServer.debug = True


@webServer.route("/")
def serve():
    """ Web root. """
    return render_template('index.html')


@webServer.route('/_send_motors', methods=["PUT", "POST"])
def send_motors():

    """ Handles AJAX requests. """

    requestJson = request.get_json()
    webServer.extensions['queue'].put(requestJson)

    retVal = {"message": "hi from Flask"}
    return jsonify(**retVal)


def webserverProcess(queue):

    """ Entry point for the web server process. """

    if not hasattr(webServer, 'extensions'):
        webServer.extensions = {}
    webServer.extensions['queue'] = queue

    webServer.run(use_reloader=False)

# ============================================================================
# Main function
# ============================================================================
def main():

    # for inter-process communication
    q = mp.Queue()

    # web server process
    p_webserver = mp.Process(target=webserverProcess, args=(q, ))
    p_webserver.start()

    # jacket renderer process
    p_renderer = mp.Process(target=rendererProcess, args=(q, ))
    p_renderer.start()


if __name__ == "__main__":
    main()
