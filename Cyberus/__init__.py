import mraa
import pyrealsense as pyrs


# UP-board pin initialization for ultrasonics
RF_TRIG = mraa.Gpio(23)
RF_ECHO = mraa.Gpio(24)
LF_TRIG = mraa.Gpio(31)
LF_ECHO = mraa.Gpio(32)

# set trigger pins as output and echo pins as input
RF_TRIG.dir(mraa.DIR_OUT)
RF_ECHO.dir(mraa.DIR_IN)
LF_TRIG.dir(mraa.DIR_OUT)
LF_ECHO.dir(mraa.DIR_IN)

# set trigger pins low
RF_TRIG.write(0)
LF_TRIG.write(0)

