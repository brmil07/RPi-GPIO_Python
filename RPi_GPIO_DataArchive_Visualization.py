import RPi.GPIO as GPIO
import time
import statistics as sts
import matplotlib.pyplot as plt
from Adafruit_LED_Backpack import SevenSegment
from collections import deque

# Initialize FIFO buffer size
q_size = 100
queue = deque(maxlen=q_size)

GPIO.setwarnings(False)
# Setting up numbering systems and inputs of the RPi
GPIO.setmode(GPIO.BCM)

# Initialize GPIO pins
TRIG = 16
ECHO = 12
btn_up = 26
btn_dn = 13
btn_left = 25
btn_right = 19
buzzer = 18

# Setting up pins as inputs or outputs
GPIO.setup(btn_up, GPIO.IN)
GPIO.setup(btn_dn, GPIO.IN)
GPIO.setup(btn_left, GPIO.IN)
GPIO.setup(btn_right, GPIO.IN)
GPIO.setup(buzzer, GPIO.OUT)

alpha = 17241.0

def show_distance_7sd(t_echo, alpha, address=0x70):
    segment_7SD = SevenSegment.SevenSegment(address)
    segment_7SD.begin()
    segment_7SD.set_brightness(0)

    distance = round(t_echo * alpha, 1)
    # Ultrasonic ranging module provides 2cm - 400cm
    ranged_distance = round(distance * 100.0 / 400.0, 1)

    str_distance = str(ranged_distance)

    if ranged_distance == 100.0:
        digit1 = int(str_distance[0])
        digit2 = int(str_distance[1])
        digit3 = int(str_distance[2])
        digit4 = int(str_distance[4])

        segment_7SD.set_digit(0, digit4)
        segment_7SD.set_digit(1, digit3)
        segment_7SD.set_digit(2, digit2)
        segment_7SD.set_digit(3, digit1)
    elif ranged_distance < 100.0 and ranged_distance >= 10.0:
        digit1 = int(str_distance[0])
        digit2 = int(str_distance[1])
        digit3 = int(str_distance[3])

        segment_7SD.set_digit(0, digit3)
        segment_7SD.set_digit(1, digit2)
        segment_7SD.set_digit(2, digit1)
        segment_7SD.set_digit(3, 0)
    elif ranged_distance < 10.0 and ranged_distance >= 1.0:
        digit1 = int(str_distance[0])
        digit2 = int(str_distance[2])

        segment_7SD.set_digit(0, digit2)
        segment_7SD.set_digit(1, digit1)
        segment_7SD.set_digit(2, 0)
        segment_7SD.set_digit(3, 0)
    else:
        digit1 = int(str_distance[2])

        segment_7SD.set_digit(0, digit1)
        segment_7SD.set_digit(1, 0)
        segment_7SD.set_digit(2, 0)
        segment_7SD.set_digit(3, 0)

    segment_7SD.set_decimal(1, True)

    segment_7SD.write_display()

def display_7sd(value):
    segment_7SD = SevenSegment.SevenSegment(0x70)
    segment_7SD.begin()
    segment_7SD.set_brightness(0)

    ranged_distance = value
    str_distance = str(ranged_distance)

    if ranged_distance == 100.0:
        digit1 = int(str_distance[0])
        digit2 = int(str_distance[1])
        digit3 = int(str_distance[2])
        digit4 = int(str_distance[4])

        segment_7SD.set_digit(0, digit4)
        segment_7SD.set_digit(1, digit3)
        segment_7SD.set_digit(2, digit2)
        segment_7SD.set_digit(3, digit1)
    elif ranged_distance < 100.0 and ranged_distance >= 10.0:
        digit1 = int(str_distance[0])
        digit2 = int(str_distance[1])
        digit3 = int(str_distance[3])

        segment_7SD.set_digit(0, digit3)
        segment_7SD.set_digit(1, digit2)
        segment_7SD.set_digit(2, digit1)
        segment_7SD.set_digit(3, 0)
    elif ranged_distance < 10.0 and ranged_distance >= 1.0:
        digit1 = int(str_distance[0])
        digit2 = int(str_distance[2])

        segment_7SD.set_digit(0, digit2)
        segment_7SD.set_digit(1, digit1)
        segment_7SD.set_digit(2, 0)
        segment_7SD.set_digit(3, 0)
    else:
        digit1 = int(str_distance[2])

        segment_7SD.set_digit(0, digit1)
        segment_7SD.set_digit(1, 0)
        segment_7SD.set_digit(2, 0)
        segment_7SD.set_digit(3, 0)

    segment_7SD.set_decimal(1, True)
    segment_7SD.write_display()

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
print("Start measuring... ")
position = -1

while 1:

    if not GPIO.input(btn_up):
        if not len(queue) == q_size:
            echo_time = get_echo_time(TRIG, ECHO)
            distance = round(echo_time * alpha, 1)
            ranged_distance = round(distance * 100.0 / 400.0, 1)
            queue.append(ranged_distance)
            show_distance_7sd(echo_time, alpha)
        else:
            echo_time = get_echo_time(TRIG, ECHO)
            queue.popleft()
            distance = round(echo_time * alpha, 1)
            ranged_distance = round(distance * 100.0 / 400.0, 1)
            queue.append(ranged_distance)
            show_distance_7sd(echo_time, alpha)

        position = -1
        time.sleep(0.5)

    if not GPIO.input(btn_left):
        if not abs(position) == len(queue):
            position = position - 1
            display_7sd(queue[position])
        else:
            display_7sd(queue[position])

        time.sleep(0.5)

    if not GPIO.input(btn_right):
        if position == -q_size or position < -1:
            position = position + 1
            display_7sd(queue[position])
        elif position > -1:
            position = -1
            display_7sd(queue[position])
        else:
            display_7sd(queue[position])

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