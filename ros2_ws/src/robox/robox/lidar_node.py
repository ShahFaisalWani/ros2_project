import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import serial

class LFCDLaser:
    def __init__(self, port, baud_rate):
        self.port = port
        self.baud_rate = baud_rate
        self.serial = serial.Serial(port=self.port, baudrate=self.baud_rate)
        self.serial.write(b"b")  # Start motor

    def poll(self):
        raw_bytes = bytearray(2520)
        start_count = 0
        got_scan = False

        while not got_scan:
            # Wait until first data sync of frame: 0xFA, 0xA0
            raw_bytes[start_count] = self.serial.read(1)[0]

            if start_count == 0:
                if raw_bytes[start_count] == 0xFA:
                    start_count = 1
            elif start_count == 1:
                if raw_bytes[start_count] == 0xA0:
                    start_count = 0
                    got_scan = True

                    # Now that entire start sequence has been found, read in the rest of the message
                    raw_bytes[2:] = self.serial.read(2518)

                    ranges = []
                    angles = []

                    for i in range(0, 2520, 42):
                        if raw_bytes[i] == 0xFA and raw_bytes[i + 1] == (0xA0 + i // 42):

                            for j in range(i + 4, i + 40, 6):
                                index = 6 * (i // 42) + (j - 4 - i) // 6

                                byte2 = raw_bytes[j + 2]
                                byte3 = raw_bytes[j + 3]

                                range_val = (byte3 << 8) + byte2
                                ranges.append(range_val / 1000.0)

                                # Assuming the angle increment is constant
                                angles.append(index * 0.0174533)  # Convert index to radians

                    return ranges, angles

    def close(self):
        self.serial.write(b"e")  # Stop motor
        self.serial.close()


class LaserScanPublisherNode(Node):
    def __init__(self):
        super().__init__('laser_scan_publisher')
        self.publisher_ = self.create_publisher(LaserScan, 'scan', 10)
        self.timer = self.create_timer(0.1, self.publish_scan)
        self.laser = LFCDLaser('/dev/ttyUSB0', 230400)

    def publish_scan(self):
        ranges, angles = self.laser.poll()

        if ranges and angles:
            scan_msg = LaserScan()
            scan_msg.header.frame_id = 'laser_frame'
            scan_msg.header.stamp = self.get_clock().now().to_msg()
            scan_msg.angle_min = min(angles)
            scan_msg.angle_max = max(angles)
            scan_msg.angle_increment = angles[1] - angles[0]
            scan_msg.range_min = min(ranges)
            scan_msg.range_max = max(ranges)
            scan_msg.ranges = ranges
            self.publisher_.publish(scan_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LaserScanPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()