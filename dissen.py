import RPi.GPIO as GPIO
import time
import signal
import sys

# Set the GPIO mode
GPIO.setmode(GPIO.BCM)

# Set up the HC-SR04 distance sensor
TRIG = 23
ECHO = 24

# Set up the L298N motor driver
MOTOR1_A = 19
MOTOR1_B = 16
MOTOR2_A = 21
MOTOR2_B = 26

# Set the desired distance between the robots
DESIRED_DISTANCE = 30

# Set the speed of the motors (0-100)
MOTOR_SPEED = 50

# Set the trigger and echo pins as input and output
GPIO.setup(TRIG,GPIO.OUT)
GPIO.setup(ECHO,GPIO.IN)

# Set the pins for the motor driver as output
GPIO.setup(MOTOR1_A,GPIO.OUT)
GPIO.setup(MOTOR1_B,GPIO.OUT)
GPIO.setup(MOTOR2_A,GPIO.OUT)
GPIO.setup(MOTOR2_B,GPIO.OUT)

# Create PWM instances for the motor driver
motor1_a = GPIO.PWM(MOTOR1_A, 100)
motor1_b = GPIO.PWM(MOTOR1_B, 100)
motor2_a = GPIO.PWM(MOTOR2_A, 100)
motor2_b = GPIO.PWM(MOTOR2_B, 100)

# Start the PWM with a duty cycle of 0
motor1_a.start(0)
motor1_b.start(0)
motor2_a.start(0)
motor2_b.start(0)

def move_forward():
  # Set the duty cycle of the motor driver to move the motors forward
  motor1_a.ChangeDutyCycle(MOTOR_SPEED)
  motor1_b.ChangeDutyCycle(0)
  motor2_a.ChangeDutyCycle(MOTOR_SPEED)
  motor2_b.ChangeDutyCycle(0)

def move_backward():
  # Set the duty cycle of the motor driver to move the motors backward
  motor1_a.ChangeDutyCycle(0)
  motor1_b.ChangeDutyCycle(MOTOR_SPEED)
  motor2_a.ChangeDutyCycle(0)
  motor2_b.ChangeDutyCycle(MOTOR_SPEED)

def stop_motors():
  # Set the duty cycle of the motor driver to stop the motors
  motor1_a.ChangeDutyCycle(0)
  motor1_b.ChangeDutyCycle(0)
  motor2_a.ChangeDutyCycle(0)
  motor2_b.ChangeDutyCycle(0)

def close(signal, frame):
  # Print a message and clean up the GPIO when the program is interrupted
  print("\nTurning off ultrasonic distance detection...\n")
  stop_motors()
  GPIO.cleanup() 
  sys.exit(0)

# Set the signal handler for the SIGINT signal (Ctrl+C)
signal.signal(signal.SIGINT, close)

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
    stop_motors()
