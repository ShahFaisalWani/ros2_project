import RPi.GPIO as GPIO
import time

# Pin setup
# pin_a = 5  # Adjust this pin number based on your wiring
# pin_b = 6  # Adjust this pin number based on your wiring
pin_a = 21 
pin_b = 20 

# Use BCM GPIO references instead of physical pin numbers
GPIO.setmode(GPIO.BCM)

# Set up the GPIO channels - one input per channel
GPIO.setup(pin_a, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(pin_b, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Variables to keep track of the count and direction
encoder_direction = 0
encoder_count = 0
last_state_a = GPIO.input(pin_a)

def encoder_callback(channel):
    global encoder_count, last_state_a
    
    # Read the current state of the encoder pins
    state_a = GPIO.input(pin_a)
    state_b = GPIO.input(pin_b)
    
    # Determine direction
    if state_a != last_state_a:  # Only count on one channel to avoid duplicates
        if state_a != state_b:
            encoder_direction = 1
        else:
            encoder_direction = -1
        encoder_count += 1
        print(f"Encoder count: {encoder_count}, Direction: {'Clockwise' if encoder_direction > 0 else 'Counter-clockwise'}")
    
    # Update last state
    last_state_a = state_a

# Add event detection on both pins
GPIO.add_event_detect(pin_a, GPIO.BOTH, callback=encoder_callback)
GPIO.add_event_detect(pin_b, GPIO.BOTH, callback=encoder_callback)

try:
    while True:
        time.sleep(1)  # Reduce CPU usage with a sleep
except KeyboardInterrupt:
    print("Stopped by User")
    GPIO.cleanup()  # Clean up GPIO on CTRL+C exit
