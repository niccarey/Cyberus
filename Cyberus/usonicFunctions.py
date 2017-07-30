import time

# Get distance based on time between trigger and echo
def getDist(TRIG, ECHO):
    TRIG.write(1)
    time.sleep(0.00001)
    TRIG.write(0)
    
    while ECHO.read() == 0:
        pulse_start = time.time()
    
    while ECHO.read() == 1:
        pulse_end = time.time()

    try:
        pulse_duration = pulse_end - pulse_start
        distance = pulse_duration * 17150
        return distance

    except:
        pass
