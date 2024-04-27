import serial
import time

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

                    good_sets = 0
                    motor_speed = 0
                    rpms = 0

                    for i in range(0, 2520, 42):
                        if raw_bytes[i] == 0xFA and raw_bytes[i + 1] == (0xA0 + i // 42):
                            good_sets += 1
                            motor_speed += (raw_bytes[i + 3] << 8) + raw_bytes[i + 2]  # accumulate count for avg. time increment
                            rpms = (raw_bytes[i + 3] << 8 | raw_bytes[i + 2]) / 10

                            for j in range(i + 4, i + 40, 6):
                                index = 6 * (i // 42) + (j - 4 - i) // 6

                                byte0 = raw_bytes[j]
                                byte1 = raw_bytes[j + 1]
                                byte2 = raw_bytes[j + 2]
                                byte3 = raw_bytes[j + 3]

                                intensity = (byte1 << 8) + byte0
                                range_val = (byte3 << 8) + byte2
                                print(f"r[{359 - index}]={range_val / 1000.0}")

                    # time_increment = motor_speed / good_sets / 1e8

    def close(self):
        self.serial.write(b"e")  # Stop motor
        self.serial.close()

def main():
    port = "/dev/ttyUSB0"
    baud_rate = 230400

    try:
        laser = LFCDLaser(port, baud_rate)
        while True:
            laser.poll()
    except serial.SerialException as ex:
        print("An exception was thrown:", ex)
    except KeyboardInterrupt:
        laser.close()


if __name__ == "__main__":
    main()