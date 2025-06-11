import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Motor driver pins (do not use for sensors)
RPWM = 26
LPWM = 19
RPWM1 = 6
LPWM1 = 5
L_EN = 21
R_EN = 20
L_EN1 = 23
R_EN1 = 24

GPIO.setup(RPWM, GPIO.OUT)
GPIO.setup(LPWM, GPIO.OUT)
GPIO.setup(RPWM1, GPIO.OUT)
GPIO.setup(LPWM1, GPIO.OUT)
GPIO.setup(L_EN, GPIO.OUT)
GPIO.setup(R_EN, GPIO.OUT)
GPIO.setup(L_EN1, GPIO.OUT)
GPIO.setup(R_EN1, GPIO.OUT)

GPIO.output(R_EN, True)
GPIO.output(L_EN, True)
GPIO.output(R_EN1, True)
GPIO.output(L_EN1, True)

FREQ = 100
DUTY = 20
T_Duty = 80
S_Duty = 50

rpwm = GPIO.PWM(RPWM, FREQ)
lpwm = GPIO.PWM(LPWM, FREQ)
rpwm1 = GPIO.PWM(RPWM1, FREQ)
lpwm1 = GPIO.PWM(LPWM1, FREQ)

rpwm.start(0)
lpwm.start(0)
rpwm1.start(0)
lpwm1.start(0)

# Ultrasonic sensor pins (do not overlap with motor pins)
TRIG_F = 25
ECHO_F = 11
TRIG_L = 22
ECHO_L = 4
TRIG_R = 17
ECHO_R = 27

for trig, echo in [(TRIG_F, ECHO_F), (TRIG_L, ECHO_L), (TRIG_R, ECHO_R)]:
    GPIO.setup(trig, GPIO.OUT)
    GPIO.setup(echo, GPIO.IN)
    GPIO.output(trig, False)

def get_distance(trig, echo):
    GPIO.output(trig, True)
    time.sleep(0.00001)
    GPIO.output(trig, False)

    pulse_start = time.time()
    pulse_end = time.time()

    timeout = time.time() + 0.04  # 40ms timeout
    while GPIO.input(echo) == 0:
        pulse_start = time.time()
        if pulse_start > timeout:
            return 999  # return large distance if timeout

    timeout = time.time() + 0.04
    while GPIO.input(echo) == 1:
        pulse_end = time.time()
        if pulse_end > timeout:
            return 999

    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150
    return round(distance, 2)

def forward():
    rpwm.ChangeDutyCycle(DUTY)
    lpwm.ChangeDutyCycle(0)
    rpwm1.ChangeDutyCycle(DUTY)
    lpwm1.ChangeDutyCycle(0)

def backward():
    rpwm.ChangeDutyCycle(0)
    lpwm.ChangeDutyCycle(DUTY)
    rpwm1.ChangeDutyCycle(0)
    lpwm1.ChangeDutyCycle(DUTY)

def turn_right():
    rpwm.ChangeDutyCycle(0)
    lpwm.ChangeDutyCycle(T_Duty)
    rpwm1.ChangeDutyCycle(S_Duty)
    lpwm1.ChangeDutyCycle(0)

def turn_left():
    rpwm.ChangeDutyCycle(T_Duty)
    lpwm.ChangeDutyCycle(0)
    rpwm1.ChangeDutyCycle(0)
    lpwm1.ChangeDutyCycle(S_Duty)

def stop_all():
    rpwm.ChangeDutyCycle(0)
    lpwm.ChangeDutyCycle(0)
    rpwm1.ChangeDutyCycle(0)
    lpwm1.ChangeDutyCycle(0)

try:
    while True:
        dist_F = get_distance(TRIG_F, ECHO_F)
        dist_L = get_distance(TRIG_L, ECHO_L)
        dist_R = get_distance(TRIG_R, ECHO_R)

        print(f"Front:{dist_F}cm, Left:{dist_L}cm, Right:{dist_R}cm")

        # Case 1
        if dist_L >= 20 and dist_R >= 20 and dist_F >= 22:
            forward()
        # Case 2
        elif dist_L >= 20 and dist_R >= 20 and dist_F < 22:
            backward()
            time.sleep(0.7)
            turn_left()
            time.sleep(0.85)
            stop_all()
        # Case 3
        elif dist_L >= 20 and dist_R < 20 and dist_F >= 22:
            backward()
            time.sleep(0.7)
            turn_left()
            time.sleep(0.85)
            stop_all()
        # Case 4
        elif dist_L >= 20 and dist_R < 20 and dist_F < 22:
            backward()
            time.sleep(0.85)
            turn_left()
            time.sleep(0.85)
            stop_all()
        # Case 5
        elif dist_L < 20 and dist_R >= 20 and dist_F >= 22:
            backward()
            time.sleep(1.0)
            turn_right()
            time.sleep(0.85)
            stop_all()
        # Case 6
        elif dist_L < 20 and dist_R >= 20 and dist_F < 22:
            backward()
            time.sleep(1.0)
            turn_right()
            time.sleep(0.85)
            stop_all()
        # Case 7
        elif dist_L < 20 and dist_R < 20 and dist_F >= 22:
            backward()
            time.sleep(0.7)
            turn_left()
            time.sleep(0.85)
            stop_all()
        # Case 8
        elif dist_L < 20 and dist_R < 20 and dist_F < 22:
            backward()
            time.sleep(0.7)
            turn_left()
            time.sleep(0.85)
            stop_all()
        else:
            stop_all()

        time.sleep(0.1)

except KeyboardInterrupt:
    print("Program terminated")

finally:
    stop_all()
    rpwm.stop()
    lpwm.stop()
    rpwm1.stop()
    lpwm1.stop()
    GPIO.cleanup()