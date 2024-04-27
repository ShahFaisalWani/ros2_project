# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# import RPi.GPIO as GPIO
# from std_msgs.msg import String 

# class DriveNode(Node):
#     def __init__(self):
#         super().__init__("drive_node")
#         self.get_logger().info("Drive Node Initialized")
#         # Set up GPIO pins for motors
#         self.setup_gpio()                 
#         self.subscription = self.create_subscription(String, 'key_input', self.key_input_callback, 10)
    
#     def key_input_callback(self, msg):
#             key = msg.data
#             print("Key = " + key)
#             if key == 'w':
#                 self.move_forward()
#             elif key == 's':
#                 self.move_backward()
#             elif key == 'a':
#                 self.turn_left()
#             elif key == 'd':
#                 self.turn_right()
#             elif key == 'q':
#                 self.stop_motors()     
#                 raise SystemExit
#             else:
#                 self.stop_motors()
        
#     def setup_gpio(self):
#         # Use GPIO numbering
#         GPIO.setmode(GPIO.BCM)
#         GPIO.setwarnings(False)
        
#         # Set up motor pins
#         self.left_motor_pins = [22, 27] 
#         self.right_motor_pins = [23, 24] 
#         self.left_en_pin = 17        # Enable pin for left motor
#         self.right_en_pin = 25        # Enable pin for right motor

#         for pin in self.left_motor_pins + self.right_motor_pins:
#             GPIO.setup(pin, GPIO.OUT)
#             GPIO.output(pin, GPIO.LOW)    
        
#         GPIO.setup(self.left_en_pin, GPIO.OUT)
#         GPIO.setup(self.right_en_pin, GPIO.OUT)
#         self.left_pwm = GPIO.PWM(self.left_en_pin, 1000)  # PWM at 1000 Hz
#         self.right_pwm = GPIO.PWM(self.right_en_pin, 1000)  # PWM at 1000 Hz
#         self.left_pwm.start(40)  # Start PWM with 30% duty cycle
#         self.right_pwm.start(40)  # Start PWM with 30% duty cycle
        
        
#     def move_forward(self):  
#         self.left_pwm.ChangeDutyCycle(40)
#         self.right_pwm.ChangeDutyCycle(40)      
#         GPIO.output(self.left_motor_pins[0], GPIO.HIGH)
#         GPIO.output(self.left_motor_pins[1], GPIO.LOW)
#         GPIO.output(self.right_motor_pins[0], GPIO.HIGH)
#         GPIO.output(self.right_motor_pins[1], GPIO.LOW)
        
#     def move_backward(self):
#         self.left_pwm.ChangeDutyCycle(40)
#         self.right_pwm.ChangeDutyCycle(40)
#         GPIO.output(self.left_motor_pins[0], GPIO.LOW)
#         GPIO.output(self.left_motor_pins[1], GPIO.HIGH)
#         GPIO.output(self.right_motor_pins[0], GPIO.LOW)
#         GPIO.output(self.right_motor_pins[1], GPIO.HIGH)
        
#     def turn_left(self):
#         self.left_pwm.ChangeDutyCycle(30)
#         self.right_pwm.ChangeDutyCycle(30)
#         GPIO.output(self.left_motor_pins[0], GPIO.LOW)
#         GPIO.output(self.left_motor_pins[1], GPIO.HIGH)
#         GPIO.output(self.right_motor_pins[0], GPIO.HIGH)
#         GPIO.output(self.right_motor_pins[1], GPIO.LOW)
    
#     def turn_right(self):
#         self.left_pwm.ChangeDutyCycle(30)
#         self.right_pwm.ChangeDutyCycle(30)
#         GPIO.output(self.left_motor_pins[0], GPIO.HIGH)
#         GPIO.output(self.left_motor_pins[1], GPIO.LOW)
#         GPIO.output(self.right_motor_pins[0], GPIO.LOW)
#         GPIO.output(self.right_motor_pins[1], GPIO.HIGH)
        
#     def stop_motors(self):
#         for pin in self.left_motor_pins + self.right_motor_pins:
#             GPIO.output(pin, GPIO.LOW)                    

# def main(args=None):
#     rclpy.init(args=args)
#     node = DriveNode()    
    
#     try:
#         rclpy.spin(node)
#     except SystemExit:
#         rclpy.logging.get_logger("Quitting").info('Done')

#     node.left_pwm.stop()
#     node.right_pwm.stop()
#     GPIO.cleanup()
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
from geometry_msgs.msg import Twist

class DriveNode(Node):
    def __init__(self):
        super().__init__("drive_node")
        self.get_logger().info("Drive Node Initialized")
        self.setup_gpio()
        self.subscription = self.create_subscription(Twist, 'cmd_vel_joy', self.twist_callback, 10)
    
    def twist_callback(self, msg):
        linear_speed = msg.linear.x
        angular_speed = msg.angular.z
        self.handle_movement(linear_speed, angular_speed)
        
    def setup_gpio(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        self.left_motor_pins = [22, 27]
        self.right_motor_pins = [23, 24]
        self.left_en_pin = 17
        self.right_en_pin = 25

        for pin in self.left_motor_pins + self.right_motor_pins:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)
        
        GPIO.setup(self.left_en_pin, GPIO.OUT)
        GPIO.setup(self.right_en_pin, GPIO.OUT)
        self.left_pwm = GPIO.PWM(self.left_en_pin, 500)
        self.right_pwm = GPIO.PWM(self.right_en_pin, 500)
        self.left_pwm.start(0)
        self.right_pwm.start(0)

    def handle_movement(self, linear, angular):
        # Constants for maximum speed settings (can be adjusted based on your hardware capabilities)
        max_speed = 100  # Maximum PWM duty cycle
        base_speed = 50  # Base speed for PWM duty cycle when moving forward/backward

        # Calculate the turning component
        turn_effect = max_speed * angular / 2  # Adjust division factor based on your robot's responsiveness

        # Calculate final motor speeds accounting for both linear and angular components
        # Correcting the application of turn_effect for proper direction turning
        left_speed = base_speed * linear + turn_effect
        right_speed = base_speed * linear - turn_effect

        # Ensure speeds do not exceed the maximum allowed by PWM
        left_speed = max(-max_speed, min(max_speed, left_speed))
        right_speed = max(-max_speed, min(max_speed, right_speed))

        # Set motor directions and speeds
        self.set_motor_speed(self.left_motor_pins, left_speed)
        self.set_motor_speed(self.right_motor_pins, right_speed)

    def set_motor_speed(self, motor_pins, speed):
        # Determine the direction based on the speed's sign
        if speed >= 0:
            GPIO.output(motor_pins[0], GPIO.HIGH)
            GPIO.output(motor_pins[1], GPIO.LOW)
        else:
            GPIO.output(motor_pins[0], GPIO.LOW)
            GPIO.output(motor_pins[1], GPIO.HIGH)

        # Use absolute speed value for PWM duty cycle
        pwm = self.left_pwm if motor_pins == self.left_motor_pins else self.right_pwm
        pwm.ChangeDutyCycle(abs(speed))

def main(args=None):
    rclpy.init(args=args)
    node = DriveNode()    
    try:
        rclpy.spin(node)
    except SystemExit:
        rclpy.logging.get_logger("Quitting").info('Done')
    finally:
        node.left_pwm.stop()
        node.right_pwm.stop()
        GPIO.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
