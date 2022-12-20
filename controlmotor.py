import RPi.GPIO as GPIO
import time

# Set up the HC-SR04 distance sensor
TRIG = 23
ECHO = 24

GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG,GPIO.OUT)
GPIO.setup(ECHO,GPIO.IN)

# Set up the L298N motor driver
ENA = 13
ENB = 20
IN1 = 19
IN2 = 16
IN3 = 21
IN4 = 26

GPIO.setup(ENA,GPIO.OUT)
GPIO.setup(ENB,GPIO.OUT)
GPIO.setup(IN1,GPIO.OUT)
GPIO.setup(IN2,GPIO.OUT)
GPIO.setup(IN3,GPIO.OUT)
GPIO.setup(IN4,GPIO.OUT)

motor1_pwm = GPIO.PWM(ENA, 500)
motor2_pwm = GPIO.PWM(ENB, 500)
motor1_pwm.start(0)
motor2_pwm.start(0)

# Set the target distance between the two robots
target_distance = 30  # cm

while True:
    # Measure the distance to the other robot using the HC-SR04
    GPIO.output(TRIG, False)
    time.sleep(0.1)

    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    while GPIO.input(ECHO) == 0:
        pulse_start = time.time()

    while GPIO.input(ECHO) == 1:
        pulse_end = time.time()

    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150
    distance = round(distance, 2)
    
    print ("Distance: %.1f cm" % distance)
    
    # If the distance is smaller than the target, move the robot backwards
    if distance < target_distance:
        # Set the motor speed and direction
        motor1_pwm.ChangeDutyCycle(50)
        motor2_pwm.ChangeDutyCycle(50)
        GPIO.output(IN1, False)
        GPIO.output(IN2, True)
        GPIO.output(IN3, False)
        GPIO.output(IN4, True)
    # If the distance is larger than the target, move the robot forwards
    elif distance > target_distance:
        # Set the motor speed and direction
        motor1_pwm.ChangeDutyCycle(50)
        motor2_pwm.ChangeDutyCycle(50)
        GPIO.output(IN1, True)
        GPIO.output(IN2, False)
        GPIO.output(IN3, True)
        GPIO.output(IN4, False)
    # If the distance is exactly the target, stop the motors
    else:
        motor1_pwm.ChangeDutyCycle(0)
        motor2_pwm.ChangeDutyCycle(0)

GPIO.cleanup()
