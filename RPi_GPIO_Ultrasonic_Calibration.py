import RPi.GPIO as GPIO
import time
import statistics as sts
import matplotlib.pyplot as plt

# Setting up numbering systems and inputs of the RPi
GPIO.setmode(GPIO.BCM)

# Initialize GPIO pins
TRIG = 16
ECHO = 12
btn_up = 26
btn_dn = 13
buzzer = 18

# Setting up pins as inputs or outputs
GPIO.setup(btn_up, GPIO.IN)
GPIO.setup(btn_dn, GPIO.IN)
GPIO.setup(buzzer, GPIO.OUT)

alpha = 17241.0

def init_ultrasonic(pin_trigger, pin_echo):
    GPIO.setup(pin_trigger, GPIO.OUT)
    GPIO.setup(pin_echo, GPIO.IN)
    GPIO.output(TRIG, False)
    print("Wait for sensor...")
    time.sleep(2)

def get_echo_time(pin_trigger, pin_echo):
    # Set trigger for 1 ms to True and reset afterwards
    GPIO.output(pin_trigger, True)
    time.sleep(0.00001)
    GPIO.output(pin_trigger, False)

    # Measure t1 as long as echo pin is FALSE
    while GPIO.input(pin_echo) == 0:
        pulse_start = time.time()
    # Measure t2 as long as echo pin is TRUE
    while GPIO.input(pin_echo) == 1:
        pulse_end = time.time()

    # Calculate pulse duration
    pulse_duration = pulse_end - pulse_start

    # Calculate and show the distance
    distance = pulse_duration
    t_echo = distance

    return t_echo

def calibrate_ultrasonic(pin_trigger, pin_echo):
    treshold = 5
    calibrations = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

    # Measure the distance 10 times to find the mean of echo time
    for i in range(10):
        # Set trigger for 1 ms to True and reset afterwards
        GPIO.output(pin_trigger, True)
        time.sleep(0.00001)
        GPIO.output(pin_trigger, False)

        # Measure t1 as long as echo pin is FALSE
        while GPIO.input(pin_echo) == 0:
            pulse_start = time.time()
        # Measure t2 as long as echo pin is TRUE
        while GPIO.input(pin_echo) == 1:
            pulse_end = time.time()

        # Calculate pulse duration
        pulse_duration = pulse_end - pulse_start

        # Calculate and show the distance
        distance = pulse_duration * 17241.0
        distance = round(distance, 2)
        t_echo = distance

        calibrations[i] = t_echo

        GPIO.output(buzzer, GPIO.HIGH)
        time.sleep(0.5)
        GPIO.output(buzzer, GPIO.LOW)

    # the alpha is only accepted if the standard deviation of the measurements is below certain value, if not alpha is -1
    if sts.stdev(calibrations) < treshold:
        avg_t_echo = sts.mean(calibrations)
        alpha = 1 / avg_t_echo * 100
    else:
        alpha = -1

    GPIO.output(buzzer, GPIO.LOW)

    plt.title("Ultrasonic Calibration Data")
    plt.xlabel("Measurements")
    plt.ylabel("Distance")
    plt.plot(calibrations)
    plt.show()

    return alpha

init_ultrasonic(TRIG, ECHO)

while 1:

    if not GPIO.input(btn_up):
        print("Distance: ", round(get_echo_time(TRIG, ECHO) * alpha, 2), " cm")
        time.sleep(0.5)

    if not GPIO.input(btn_dn):
        alpha = calibrate_ultrasonic(TRIG, ECHO)
        if not alpha == -1:
            alpha = alpha
            print("Alpha is set to: ", alpha)
        else:
            print("Error: Alpha is - 1")
            alpha = 17241.0

        time.sleep(0.5)

GPIO.cleanup