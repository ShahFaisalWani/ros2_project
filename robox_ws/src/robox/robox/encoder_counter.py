import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO

class EncoderCounter(Node):
    def __init__(self):
        super().__init__('encoder_counter')
        self.encoder_pin = 5  # Set to your encoder's channel A pin
        # self.encoder_pin = 20  # Set to your encoder's channel A pin
        self.ticks = 0

        # Set up GPIO pin
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.encoder_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        # Add event detection for rising/falling edges on the encoder pin
        GPIO.add_event_detect(self.encoder_pin, GPIO.BOTH, callback=self.count_ticks)

        self.get_logger().info("Rotate the wheel one full revolution and watch the tick count.")
        self.create_timer(1.0, self.publish_tick_count)

    def count_ticks(self, channel):
        self.ticks += 1  # Increment tick count for each pulse detected

    def publish_tick_count(self):
        self.get_logger().info(f"Tick count: {self.ticks}")

    def stop(self):
        GPIO.cleanup()

def main(args=None):
    rclpy.init(args=args)
    node = EncoderCounter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
