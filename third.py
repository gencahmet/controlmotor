import RPi.GPIO as GPIO
import time

# Set up the GPIO pins for the HC-SR04 distance sensor
TRIG_PIN = 17
ECHO_PIN = 27

GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG_PIN, GPIO.OUT)
GPIO.setup(ECHO_PIN, GPIO.IN)

# Set up the GPIO pins for the L298N motor driver
MOTOR1_ENABLE_PIN = 22
MOTOR1_IN1_PIN = 18
MOTOR1_IN2_PIN = 16
MOTOR2_ENABLE_PIN = 23
MOTOR2_IN1_PIN = 12
MOTOR2_IN2_PIN = 25
GPIO.setup(MOTOR1_ENABLE_PIN, GPIO.OUT)
GPIO.setup(MOTOR1_IN1_PIN, GPIO.OUT)
GPIO.setup(MOTOR1_IN2_PIN, GPIO.OUT)
GPIO.setup(MOTOR2_ENABLE_PIN, GPIO.OUT)
GPIO.setup(MOTOR2_IN1_PIN, GPIO.OUT)
GPIO.setup(MOTOR2_IN2_PIN, GPIO.OUT)

# Set the speed of the motors (0-100)
MOTOR1_SPEED = 50
MOTOR2_SPEED = 50

# Set the target distance between the two robots (in cm)
TARGET_DISTANCE = 30

# Set up the PWM outputs for the motor driver
motor1_pwm = GPIO.PWM(MOTOR1_ENABLE_PIN, 500)
motor2_pwm = GPIO.PWM(MOTOR2_ENABLE_PIN, 500)

def measure_distance():
  # Send a trigger pulse to the HC-SR04 distance sensor
  GPIO.output(TRIG_PIN, True)
  time.sleep(0.00001)
  GPIO.output(TRIG_PIN, False)

  # Measure the duration of the echo pulse
  start_time = time.time()
  end_time = time.time()
  while GPIO.input(ECHO_PIN) == 0:
    start_time = time.time()
  while GPIO.input(ECHO_PIN) == 1:
    end_time = time.time()

  # Calculate the distance based on the duration of the echo pulse
  duration = end_time - start_time
  distance = (duration * 34300) / 2

  return distance

def move_forward():
  # Set the motor driver to drive the motors forward
  GPIO.output(MOTOR1_IN1_PIN, True)
  GPIO.output(MOTOR1_IN2_PIN, False)
  GPIO.output(MOTOR2_IN1_PIN, True)
  GPIO.output(MOTOR2_IN2_PIN, False)

  # Set the motor speeds
  motor1_pwm.start(MOTOR1_SPEED)
  motor2_pwm.start(MOTOR2_SPEED)

def move_backward():
  # Set the motor driver to drive the motors backward
  GPIO.output(MOTOR1_IN1_PIN, False)
  GPIO.output(MOTOR1_IN2_PIN, True)
  GPIO.output(MOTOR2_IN1_PIN, False)
  GPIO.output(MOTOR2_IN2_PIN, True)

  # Set the motor speeds
  motor1_pwm.start(MOTOR1_SPEED)
  motor2_pwm.start(MOTOR2_SPEED)

def stop_motors():
  # Stop the motors
  motor1_pwm.stop()
  motor2_pwm.stop()

try:
  while True:
    # Measure the distance between the two robots
    distance = measure_distance()

    # Control the motor speeds according to the distance
    if distance < TARGET_DISTANCE:
      move_backward()
    elif distance > TARGET_DISTANCE:
      move_forward()
    else:
      stop_motors()

except KeyboardInterrupt:
  # Clean up the GPIO pins when the program is interrupted
  GPIO.cleanup()

