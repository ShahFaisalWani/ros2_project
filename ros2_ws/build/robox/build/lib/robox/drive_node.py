#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
from std_msgs.msg import String 

class DriveNode(Node):
    def __init__(self):
        super().__init__("drive_node")
        self.get_logger().info("Drive Node Initialized")
        # Set up GPIO pins for motors
        self.setup_gpio()                 
        self.subscription = self.create_subscription(String, 'key_input', self.key_input_callback, 10)
    
    def key_input_callback(self, msg):
            key = msg.data
            print("Key = " + key)
            if key == 'w':
                self.move_forward()
            elif key == 's':
                self.move_backward()
            elif key == 'a':
                self.turn_left()
            elif key == 'd':
                self.turn_right()
            elif key == 'q':
                self.stop_motors()     
                raise SystemExit
            else:
                self.stop_motors()
        
    def setup_gpio(self):
        # Use GPIO numbering
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        # Set up motor pins
        self.motor1_pins = [23, 24]  # Motor 1 pins
        self.motor2_pins = [22, 27]  # Motor 2 pins

        for pin in self.motor1_pins + self.motor2_pins:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)    
        
        
    def move_forward(self):
        GPIO.output(self.motor1_pins[0], GPIO.HIGH)
        GPIO.output(self.motor1_pins[1], GPIO.LOW)
        GPIO.output(self.motor2_pins[0], GPIO.HIGH)
        GPIO.output(self.motor2_pins[1], GPIO.LOW)
        
    def move_backward(self):
        GPIO.output(self.motor1_pins[0], GPIO.LOW)
        GPIO.output(self.motor1_pins[1], GPIO.HIGH)
        GPIO.output(self.motor2_pins[0], GPIO.LOW)
        GPIO.output(self.motor2_pins[1], GPIO.HIGH)

    def turn_left(self):
        GPIO.output(self.motor1_pins[0], GPIO.HIGH)
        GPIO.output(self.motor1_pins[1], GPIO.LOW)
        GPIO.output(self.motor2_pins[0], GPIO.LOW)
        GPIO.output(self.motor2_pins[1], GPIO.HIGH)
        
    def turn_right(self):
        GPIO.output(self.motor1_pins[0], GPIO.LOW)
        GPIO.output(self.motor1_pins[1], GPIO.HIGH)
        GPIO.output(self.motor2_pins[0], GPIO.HIGH)
        GPIO.output(self.motor2_pins[1], GPIO.LOW)
        
    def stop_motors(self):
        for pin in self.motor1_pins + self.motor2_pins:
            GPIO.output(pin, GPIO.LOW)                    

def main(args=None):
    rclpy.init(args=args)
    node = DriveNode()    
    
    try:
        rclpy.spin(node)
    except SystemExit:
        rclpy.logging.get_logger("Quitting").info('Done')

    GPIO.cleanup()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
