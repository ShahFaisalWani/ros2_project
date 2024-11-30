# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import LaserScan
# import serial
# from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
# import time

# class LFCDLaser:
#     def __init__(self, port, baud_rate):
#         self.port = port
#         self.baud_rate = baud_rate
#         self.serial = serial.Serial(port=self.port, baudrate=self.baud_rate)
#         self.serial.write(b"b")  # Start motor

#     def poll(self):
#         raw_bytes = bytearray(2520)
#         start_count = 0
#         got_scan = False

#         while not got_scan:
#             # Wait until first data sync of frame: 0xFA, 0xA0
#             raw_bytes[start_count] = self.serial.read(1)[0]

#             if start_count == 0:
#                 if raw_bytes[start_count] == 0xFA:
#                     start_count = 1
#             elif start_count == 1:
#                 if raw_bytes[start_count] == 0xA0:
#                     start_count = 0
#                     got_scan = True

#                     # Now that entire start sequence has been found, read in the rest of the message
#                     raw_bytes[2:] = self.serial.read(2518)

#                     ranges = []
#                     angles = []
#                     intensities = []

#                     for i in range(0, 2520, 42):
#                         if raw_bytes[i] == 0xFA and raw_bytes[i + 1] == (0xA0 + i // 42):

#                             for j in range(i + 4, i + 40, 6):
#                                 index = 6 * (i // 42) + (j - 4 - i) // 6
#                                 byte2 = raw_bytes[j + 2]
#                                 byte3 = raw_bytes[j + 3]
#                                 byte4 = raw_bytes[j + 4]
#                                 byte5 = raw_bytes[j + 5]

#                                 range_val = (byte3 << 8) + byte2
#                                 intensity_val = (byte5 << 8) + byte4

#                                 ranges.append(range_val / 1000.0)
#                                 intensities.append(float(intensity_val))

#                                 # Assuming the angle increment is constant
#                                 angles.append(index * 0.0174533)  # Convert index to radians

#                     return ranges, angles, intensities

#     def close(self):
#         self.serial.write(b"e")  # Stop motor
#         self.serial.close()


# class LaserScanPublisherNode(Node):
#     def __init__(self):
#         super().__init__('laser_scan_publisher')
#         qos_profile = QoSProfile(
#             reliability=ReliabilityPolicy.BEST_EFFORT,
#             history=HistoryPolicy.KEEP_LAST,
#             depth=10  # Queue depth
#         )
#         self.publisher_ = self.create_publisher(LaserScan, 'scan', qos_profile)
#         self.timer = self.create_timer(0.1, self.publish_scan)
#         self.laser = LFCDLaser('/dev/ttyUSB0', 230400)

#     def publish_scan(self):
#         ranges, angles, intensities = self.laser.poll()

#         if ranges and angles:
#             # Ensure that we have 360 readings
#             num_readings = 360
#             angle_increment = 2 * 3.141592 / num_readings

#             if len(ranges) < num_readings:
#                 # Fill missing readings with max range (or you can interpolate if necessary)
#                 ranges.extend([float('inf')] * (num_readings - len(ranges)))
#                 intensities.extend([0.0] * (num_readings - len(intensities)))

#             # Reverse ranges to fix the inverted scan issue
#             ranges.reverse()

#             scan_msg = LaserScan()
#             scan_msg.header.frame_id = 'laser_frame'
#             scan_msg.header.stamp = self.get_clock().now().to_msg()
#             scan_msg.angle_min = -3.141592  # Start from -180 degrees
#             scan_msg.angle_max = 3.141592   # End at +180 degrees
#             scan_msg.angle_increment = angle_increment
#             scan_msg.range_min = 0.12  # Minimum value to consider (120mm)
#             scan_msg.range_max = 3.5   # Maximum value to consider (3500mm)

#             # Filter ranges for min/max values
#             filtered_ranges = [
#                 r if 0.12 <= r <= 3.5 else float('inf') for r in ranges
#             ]

#             scan_msg.ranges = filtered_ranges
#             scan_msg.intensities = intensities
#             self.publisher_.publish(scan_msg)


# def main(args=None):
#     rclpy.init(args=args)
#     node = LaserScanPublisherNode()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass

#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import serial
import time
import numpy as np
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import threading  # Added for threading support

class LidarNode(Node):
    def __init__(self):
        super().__init__('lidar_node')

        # Parameters
        # self.port = "/dev/ttyUSB0"
        self.port = "/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3:1.0-port0"
        self.baud_rate = 230400

        # Create LaserScan publisher
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)

        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.publish_static_transform()

        # Initialize lock for thread safety
        self.scan_lock = threading.Lock()

        # Connect to the LIDAR
        try:
            self.lidar_serial = serial.Serial(self.port, self.baud_rate)
            self.get_logger().info(f"LIDAR connected: {self.lidar_serial.isOpen()}")
        except serial.serialutil.SerialException:
            self.get_logger().error("LIDAR connection error...")
            self.destroy_node()
            return

        # Start polling LIDAR data in a separate thread
        self.lidar_thread = threading.Thread(target=self.poll_lidar_data)
        self.lidar_thread.daemon = True
        self.lidar_thread.start()

        # Create a timer to publish the data every 20 ms
        self.publish_timer = self.create_timer(0.02, self.publish_scan)

    def publish_static_transform(self):
        static_transform = TransformStamped()

        static_transform.header.stamp = self.get_clock().now().to_msg()
        static_transform.header.frame_id = 'base_footprint'
        static_transform.child_frame_id = 'laser_frame'

        # Set the translation and rotation as per your sensor's actual position
        static_transform.transform.translation.x = 0.0
        static_transform.transform.translation.y = 0.0
        static_transform.transform.translation.z = 0.0

        static_transform.transform.rotation.x = 0.0
        static_transform.transform.rotation.y = 0.0
        static_transform.transform.rotation.z = 0.0
        static_transform.transform.rotation.w = 1.0

        self.static_broadcaster.sendTransform(static_transform)
        self.get_logger().info("Published static transform from 'base_link' to 'laser_frame'.")

    def poll_lidar_data(self):
        start_count = 0
        raw_bytes = np.zeros(2520, dtype='bytes')

        while rclpy.ok():
            try:
                read_byte = self.lidar_serial.read(1)

                if start_count == 0:
                    if read_byte == b'\xfa':
                        start_count = 1
                        byte0 = read_byte
                else:
                    if start_count == 1:
                        byte1 = read_byte
                        if read_byte == b'\xa0':
                            start_count = 0
                            received = self.lidar_serial.read(2518)
                            raw_bytes = bytearray()
                            raw_bytes.extend(byte0)
                            raw_bytes.extend(byte1)
                            raw_bytes.extend(received)
                            self.process_lidar_data(raw_bytes)
                    else:
                        start_count = 0

            except Exception as e:
                self.get_logger().error(f"LIDAR polling exception: {e}")
                time.sleep(2)

    def process_lidar_data(self, raw_bytes):
        scan_msg = LaserScan()
        scan_msg.header.frame_id = 'laser_frame'
        scan_msg.angle_min = 0.0
        scan_msg.angle_max = 2.0 * np.pi
        scan_msg.angle_increment = 2.0 * np.pi / 360.0
        scan_msg.range_min = 0.12
        scan_msg.range_max = 3.5
        scan_msg.ranges = [0.0] * 360
        scan_msg.intensities = [0.0] * 360

        for i in range(0, len(raw_bytes), 42):
            if raw_bytes[i] == 250 and raw_bytes[i + 1] == (160 + i // 42):
                for j in range(i + 4, i + 40, 6):
                    index = int(6 * (i / 42) + (j - 4 - i) / 6)
                    byte0 = raw_bytes[j]
                    byte1 = raw_bytes[j + 1]
                    byte2 = raw_bytes[j + 2]
                    byte3 = raw_bytes[j + 3]

                    intensity = (byte1 << 8) + byte0
                    ranges = (byte3 << 8) + byte2

                    scan_msg.ranges[359 - index] = ranges / 1000.0
                    scan_msg.intensities[359 - index] = intensity

        # Update the timestamp
        scan_msg.header.stamp = self.get_clock().now().to_msg()

        # Store the latest scan message with thread safety
        with self.scan_lock:
            self.latest_scan_msg = scan_msg

    def publish_scan(self):
        with self.scan_lock:
            if hasattr(self, 'latest_scan_msg'):
                # Update the timestamp before publishing
                self.latest_scan_msg.header.stamp = self.get_clock().now().to_msg()
                self.scan_pub.publish(self.latest_scan_msg)
                # self.get_logger().info(f"Published LaserScan message with {len(self.latest_scan_msg.ranges)} ranges.")

def main(args=None):
    rclpy.init(args=args)
    lidar_node = LidarNode()
    rclpy.spin(lidar_node)
    lidar_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()