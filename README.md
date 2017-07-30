# Cyberus
Scaffolding code for Cyberus project (to be updated end August)

Cyberus is a small skeleton package to manage sensor/motor/navigation functions for the Cyberus mobile robot project. It uses the Adafruit R-PI motor hat, Realsense Python wrapper (pyrealsense), and some small generic ultrasonic sensors. Resources for setting these up can be found at the bottom of the ReadMe.

An example control code demonstrates how to adjust motor speeds based on feedback from the RGBD sensor:

*Example high level control* 
The robot tries to keep moving forward at a steady speed, while avoiding obstacles. The image processing subfunctions return an error metric (yaw_error) which is proportional to the angular distance between the desired heading and the current heading. To steer, a proportional-derivative function translates this heading error into desired wheel speeds. To reduce processor load and oversteering, the heading error is only sampled every five frames.

  *Motors:*
  Motor speeds are fed through a lowpass filter, to reduce motor loading when a sudden speed or directional change is called for. The low-level motor control function takes inputs vdLeft, vdRight, which are the desired speeds of the left and right hand sides accordingly. This function could easily be expanded to control all four wheels independently. Note that because the Adafruit motor control drivers take unsigned input, we must be careful about which direction we drive the motors in (BACKWARDS or FORWARDS).

  *RealSense:* 
  Visual control is enabled through the RealSense depth camera. 
  There are two depth filters currently implemented, as demonstration functions. 
  'depthmap_seg_nav' finds image regions, identifies large regions which are further away, calculates the centroid, and returns an error signal proportional to the offset between this centroid and the camera image centre (along the x-axis). 
  'depth_flow_nav' (which is a little more robust, and probably faster), low-pass filters the depth image to generate a pseudo-depth-flowmap, and cross correlates this with a basic gaussian template to look for a globally maximum flow peak. It returns the heading error between the current heading and the location of this global maxima. 

  Both of these functions are quite crude, and easily confused!

  *Remote Streaming:*
  The realsense colour and depth frames are streamed to a web socket, which can be accessed in a browser at ip.address.ofthe.robot:5000. Over ethernet, this streaming is close to real time, however when the robot is untethered, the wifi connection throttles the data significantly, so we send only every 30th frame (roughly 1 frame a second). 
  This process operates in parallel with the navigation loop, so slower frame output should not interfere with the robot performance.

  However for best performance, it is recommended to disable remote streaming and stream to browser and just display the image locally, as normal.


  Some basic functionality that could be added quickly:

    Stop or turn around when a dead-end is reached
    high speed tight turns (stop or reverse one set of motors while speeding up the other side significantly)
    size analysis of segmented depth field: can we fit through a gap?

# Resources

*Cyberus Hardware*
- UP board 4GB RAM+ 32 GB eMMC: http://up-shop.org/up-boards/43-up-board-4b-32-gb-emmc-memory.html
- Intel RealSense R200: https://software.intel.com/en-us/realsense/r200camera
- Adafruit 16-Channel PWM / Servo HAT for Raspberry Pi: https://www.adafruit.com/products/2327
- Adafruit DC & Stepper Motor HAT for Raspberry Pi: https://www.adafruit.com/products/2348
- Small Robot Chassis: https://www.amazon.com/UniHobby-Chassis-Maximum-Aluminum-Projects/dp/B01IRKKGKI/ref=sr_1_6?ie=UTF8&qid=1491499571&sr=8-6&keywords=Smart+Car+Chassis+4WD
- Large chassis (Wild Thumper): https://www.pololu.com/product/1565
- Pololu 5v/5a Step-down Voltage Regulator: https://www.pololu.com/product/2851
- Floureon 7.4V 5200mAh High Power 2S 30C Lipo Battery (motors): https://www.amazon.com/gp/product/B00HWQ1JAU/ref=oh_aui_detailpage_o03_s00?ie=UTF8&psc=1
- Gens ace LiPo Battery Pack 2200mAh 25C 3S 11.1V (computer): https://www.amazon.com/gp/product/B00WJN4LG0/ref=oh_aui_detailpage_o04_s00?ie=UTF8&psc=1
- CanaKit Raspberry Pi WiFi Wireless Adapter / Dongle (802.11 n/g/b 150 Mbps): https://www.amazon.com/gp/product/B00GFAN498/ref=oh_aui_detailpage_o04_s00?ie=UTF8&psc=1
- Deans Style Connectors (batteries):https://www.amazon.com/gp/product/B01CWYFS4Y/ref=oh_aui_detailpage_o01_s01?ie=UTF8&psc=1
- Board Offsets (M2.5): https://www.amazon.com/gp/product/B06XXV8RTR/ref=oh_aui_detailpage_o00_s00?ie=UTF8&psc=1
- USB 3.0 Adapter: https://www.amazon.com/gp/product/B01BVR7I9G/ref=oh_aui_detailpage_o01_s01?ie=UTF8&psc=1
- USB 3.0 Adapter Cable: https://www.amazon.com/gp/product/B00II0H7T6/ref=oh_aui_detailpage_o01_s00?ie=UTF8&psc=1
- Raspberry Pi Camera: https://www.raspberrypi.org/documentation/raspbian/applications/camera.md

*Setting up the UP Board:*

The UP board does not come with an operating system installed. We are currently running Ubuntu 16.04 with an UP board specific Kernel.

To setup the UP board, follow these steps in order.

1. Install Ubuntu 16.04 using a bootable usb drive (Daniel has a drive available, or a new drive can be made. If making a new drive, it is easiest to do from a Linux machine.)

2. Install Librealsense with necessary patches. Librealsense is a cross platform API for the Real Sense depth camera.

    The GitHub repository for Librealsense can be found here: https://github.com/IntelRealSense/librealsense
    To install librealsense, clone or download the repository. Then, follow the instructions in the installation guide here: https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md

3. Install pyrealsense, which is a python wrapper for the Librealsense API.

    The GitHub repository for pyrealsense is here: https://github.com/toinsson/pyrealsense
    Best way to install pyrealsense is to use pip as root
        sudo pip install pyrealsense
        pip should already be installed with python, but if not you may have to install setuptools and wheel

4. Install OpenCV computer vision software

    Easiest way to install OpenCV is to use an install script
    create a file "install-opencv.sh"
        emacs install-opencv.sh
        Copy the script here into the new file and save: https://github.com/milq/milq/blob/master/scripts/bash/install-opencv.sh
    Run the install script- this will take some time
        bash install-opencv.sh

5. Install QT Creator development IDE (This needs to be updated with final file association steps)

    Follow installation guide here: https://wiki.qt.io/Install_Qt_5_on_Ubuntu

6. Install the UP board linux kernel for Ubuntu 16.04

    Installation instructions can be found on the UP wiki here: https://up-community.org/wiki/Ubuntu

7. Remove generic linux kernels

    This ensures the board boots using the UP board kernel
    Instructions for removing the generic kernel are here: https://up-community.org/wiki/Ubuntu

*Using the AdaFruit DC & Stepper Motor Hat:*

The Adafruit DC & Stepper Motor Hat is found here: https://www.adafruit.com/products/2348

1. Download and install the AdaFruit python code that has been modified by Emutex for the Up Board

    Install the "Adafruit_GPIO" library following the instructions at the top of the page here under "overview": https://up-community.org/wiki/Adafruit
    Install the "Adafruit Motor HAT Python library" following instructions here: https://up-community.org/wiki/Adafruit#Using_Adafruit_DC_.26_Stepper_Motor_HAT_on_UP

2. Solder headers and terminal blocks as shown here: https://learn.adafruit.com/adafruit-dc-and-stepper-motor-hat-for-raspberry-pi

3. Connect to power supply or battery: https://learn.adafruit.com/adafruit-dc-and-stepper-motor-hat-for-raspberry-pi/powering-motors

4. Connect appropriate DC motor (NOTE: example code DCTest.py works with block M3, not M1): https://learn.adafruit.com/adafruit-dc-and-stepper-motor-hat-for-raspberry-pi/using-dc-motors

5. To run an example, follow the directions here: https://up-community.org/wiki/Adafruit#Using_Adafruit_DC_.26_Stepper_Motor_HAT_on_UP

    NOTE: examples must be run as root: sudo python DCTest.py

*Using the AdaFruit 16 Channel PWM/Servo Hat:*

The Adafruit 16 Channel PWM/Servo Hat is found here: https://www.adafruit.com/products/2327

1. Download and install the AdaFruit python code that has been modified by Emutex for the Up Board

    Install the "Adafruit_GPIO" library following the instructions at the top of the page here under "overview": https://up-community.org/wiki/Adafruit
    Install the "Adafruit_PCA9685" Python library following instructions here: https://up-community.org/wiki/Adafruit#Using_Adafruit_16-Channel_PWM_.2F_Servo_HAT_on_UP

2. Install MRAA which is needed to manage some I2C communication tasks

    MRAA GiHub repository is here: https://github.com/intel-iot-devkit/mraa
    Follow the instructions under "Installing on Ubuntu"

3. Solder headers and terminal blocks as shown here: https://learn.adafruit.com/adafruit-16-channel-pwm-servo-hat-for-raspberry-pi/

4. Connect to power supply or battery: https://learn.adafruit.com/adafruit-16-channel-pwm-servo-hat-for-raspberry-pi/powering-servos

5. Connect appropriate servo: https://learn.adafruit.com/adafruit-16-channel-pwm-servo-hat-for-raspberry-pi/connecting-servoss

6. To run an example, follow the directions here: https://up-community.org/wiki/Adafruit#Using_Adafruit_16-Channel_PWM_.2F_Servo_HAT_on_UP

    NOTE: examples must be run as root: sudo python simpletest.py

*Talking to the knock-off PS2 controller*

How to talk to the USB controller:
1. Type lsusb in a terminal window and take a note of how many devices are already connected
2. Plug in controller
3. type lsusb again and see if something new has popped up - should be called "game controller" or DragonRise or similar
4. Install the linux joystick package:
sudo apt-get install joystick

You can use the joystick package to talk to the controller. Easiest way to make sure things are working is:
jstest /dev/input/js0

You can also get a list of device addresses with
jscal -q /dev/input/js0

and just type
jscal
to get a list of more options that may help you map the controller. jscal -c, for example, will try to calibrate the device (unclear how successful this will be)

5. If you want to work with raw USB data, you will probably want to install the python USB command interface:
sudo pip install PyUSB
This should not be necessary to talk to the controller, though. See me for example code if you get very stuck.
