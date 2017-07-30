#!/usr/bin/env python
# -*- coding: utf-8 -*-

from flask import Flask, render_template, Response
from pythonds.basic.stack import Stack
from collections import deque

import logging
import time
import numpy as np
import cv2
import thread

import atexit

""" As an example, I've set up a cyberus package to manage sensor/motor/navigation functions. See: import Cyberus. You don't have to use this structure, but try to keep sensor and motor functions grouped so we can easily find them. """

import Cyberus   # sets up any initialization functions/constants
from Cyberus.motorFunctions import MotorStatus
from Cyberus.realsenseNavFunctions import NavData

logging.basicConfig(level=logging.INFO)
atexit.register(turnOffMotors)

### INITIALISING ###

# image display stack
imQueue = deque()


### WEB STREAMING FUNCTIONS ###

# set up flask app for streaming to browser
app = Flask(__name__)

@app.route('/')
def index():
    return render_template('./index.html')

def gen():
    # pops image from queue, displays in browser frame.
    # streaming over wifi slows things down massively. Output once per second to reduce lag
    global imQueue
    imcount = 0
    imint = 30
    
    while True:
        try:
            imcount += 1
            imshow = imQueue.popleft()
            if imcount> imint:
                ret, frame = cv2.imencode('.jpg', imshow)
                jpeg_encode = frame.tobytes()
                yield(b'--frame\r\n'
                      b'Content-Type: image/jpeg\r\n\r\n' + jpeg_encode + b'\r\n\r\n')
                imcount = 1
        
        except:
            pass


@app.route('/video_feed')
def video_feed():
    # triggers pop function
    return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')


def threaded_stream():
    # we use a thread to serve images remotely, to try and decrease lag
    app.run(host='0.0.0.0', port=5000,  debug=False, threaded=True)


def shutdown_server(environ):
    # graceful shutdown for external streaming
    if not 'werkzeug.server.shutdown' in environ:
        raise RuntimeError('Not running the development server')
    
    environ['werkzeug.server.shutdown']()



# -- MAIN CONTROL LOOP

if __name__ == '__main__':
    
    """ Enabling remote view will stream the sensor output to a web server
        (address is ip of robot, check before enabling, usually 10.251.209.161)
        
        Pros: can view in browser at ip.address.here:5000
        Cons: can slow down performance, depending on wifi strength. """
    
    global imQueue
    
    REMOTE_VIEW = False
    
    imwidth = 640
    
    ### INITIALISE PID VARIABLES ###
    vt = 255    # max speed
    v0 = 0      # start speed
    kp = 1      # proportional gain
    kd = 0.3    # derivative gain
    
    ## INITIALISE NAVIGATION AND STOPPING VARIABLES ###
    frameint = 5
    framecount = 0
    wallcount = 0
    timeout = 0
    yaw_e_prev = 0

    dt = 0.03*frameint     # time step for derivative control
    
    
    ## INITIALISE MOTORS
    # create a default motor object, no changes to I2C address or frequency
    mh = Adafruit_MotorHAT(addr=0x60)
    
    # create a MotorStatus instance for each motor
    motor1 = MotorStatus(mh, 1)
    motor2 = MotorStatus(mh, 2)
    motor3 = MotorStatus(mh, 3)
    motor4 = MotorStatus(mh, 4)
    
    motor_running = True

    if REMOTE_VIEW:
        # start a remote streaming thread for images
        thread.start_new_thread(threaded_stream, ())


    # start streaming data from the realsense
    pyrs.start()
    py_dev = pyrs.Device(device_id = 0, streams = [pyrs.ColourStream(fps = 30), pyrs.DepthStream(fps=30)])

   # Set up cyberus navigation class instance
    cy_nav = NavData()
    
    while motor_running:
        timethen = time.time()
        framecount += 1
        
        ## IMAGE PROCESSING
        py_dev.wait_for_frame()
        c_im = py_dev.colour
        rgb_im = c_im[..., ::-1]
        
        d_im = py_dev.depth * 0.05
        d_im_col = cv2.applyColorMap(d_im.astype(np.uint8), cv2.COLORMAP_HOT)

        # two nav options: segmentation or gradient descent. Segmentation is a problem with very noisy images
        if framecount > frameint:
            yaw_e_prev = cy_nav.yaw_error
            # cy_nav.depthmap_seg_nav(d_im_col, imwidth)
            cy_nav.depthmap_flow_nav(d_im, imwidth)
            framecount = 1

        cv2.circle(d_im_col, (cX, cY), 7, (255, 255, 255), -1)
        cv2.putText(d_im_col, str(yaw_error), (cX - 20, cY - 20),
        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        cd = np.concatenate((rgb_im, d_im_col), axis=1)
                    
        if REMOTE_VIEW:
            # push to stack:
            imQueue.append(cd)

        else:
            # I do not recommend using local display over wifi
            # but if connected to ethernet, use REMOTE_VIEW=False for local window streaming
            cv2.imshow('', cd)


        ### NAVIGATION
        if abs(cy_nav.yaw_error) > 0:
            # basic PD motor control, kp and kd not tuned
            yaw_dev = (cy_nav.yaw_error - yaw_e_prev)*dt
            vd_left = vt + kp * cy_nav.yaw_error + kd*yaw_dev   # desired speed from left motors
            vd_right = vt - kp * cy_nav.yaw_error - kd*yaw_dev  # desired speed from right motor
            
            # update motor speeds
            motor1.motorControl(vd_right)
            motor2.motorControl(vd_right)
            motor3.motorControl(vd_left)
            motor4.motorControl(vd_left)


        # try to stop if you are running into a wall
        if est_dist < 10:
            wallcount += 1
            timeout = 0
        else:
            timeout += 1
                    
        if timeout > 2:
            wallcount = 0
                            
        if wallcount > 8:
            motorrunning = False
            break
                            

        if cv2.waitKey(1) & 0xFF == ord('q'):
            motor_running=False
            break

        timenow = time.time()


    print ("ended, stopping all motors")
    
    
    motor1.turnOffMotor
    motor2.turnOffMotor
    motor3.turnOffMotor
    motor4.turnOffMotor

#shutdown_server(self)
