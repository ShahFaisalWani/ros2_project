import RPi.GPIO as GPIO
import time

# Set up GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Motor pins
LEFT_MOTOR_PINS = [22, 27]
RIGHT_MOTOR_PINS = [23, 24]
LEFT_EN_PIN = 17
RIGHT_EN_PIN = 25

# Set up the motor pins
for pin in LEFT_MOTOR_PINS + RIGHT_MOTOR_PINS:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.LOW)

GPIO.setup(LEFT_EN_PIN, GPIO.OUT)
GPIO.setup(RIGHT_EN_PIN, GPIO.OUT)

# Set up PWM for speed control
left_pwm = GPIO.PWM(LEFT_EN_PIN, 1000)  # 1000 Hz frequency
right_pwm = GPIO.PWM(RIGHT_EN_PIN, 1000)  # 1000 Hz frequency
left_pwm.start(0)  # Start with 0% duty cycle
right_pwm.start(0)  # Start with 0% duty cycle

def set_motors_direction(forward):
    if forward:
        GPIO.output(LEFT_MOTOR_PINS[0], GPIO.HIGH)
        GPIO.output(LEFT_MOTOR_PINS[1], GPIO.LOW)
        GPIO.output(RIGHT_MOTOR_PINS[0], GPIO.HIGH)
        GPIO.output(RIGHT_MOTOR_PINS[1], GPIO.LOW)
    else:
        GPIO.output(LEFT_MOTOR_PINS[0], GPIO.LOW)
        GPIO.output(LEFT_MOTOR_PINS[1], GPIO.HIGH)
        GPIO.output(RIGHT_MOTOR_PINS[0], GPIO.LOW)
        GPIO.output(RIGHT_MOTOR_PINS[1], GPIO.HIGH)

def set_motors_speed(speed):
    left_pwm.ChangeDutyCycle(speed)
    right_pwm.ChangeDutyCycle(speed)

def test_motors():
    print("Testing both motors")
    
    # Spin forward for 2 seconds
    print("Spinning forward")
    set_motors_direction(True)
    set_motors_speed(50)  # 50% speed
    time.sleep(2)
    
    # Stop for a moment
    print("Stopping")
    set_motors_speed(0)
    time.sleep(0.5)
    
    # Spin backward for 2 seconds
    print("Spinning backward")
    set_motors_direction(False)
    set_motors_speed(50)  # 50% speed
    time.sleep(2)
    
    # Stop
    print("Stopping")
    set_motors_speed(0)

try:
    test_motors()

except KeyboardInterrupt:
    print("Test interrupted")

finally:
    # Clean up
    left_pwm.stop()
    right_pwm.stop()
    GPIO.cleanup()
    print("Test completed")