# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# import sys
# import termios
# import tty
# import select
# import time
# from std_msgs.msg import String

# DEBOUNCE_DELAY = 0.5

# def key_listener(timeout=0):
#     fd = sys.stdin.fileno()
#     old_settings = termios.tcgetattr(fd)
#     try:
#         tty.setcbreak(fd)
#         if timeout:
#             rlist, _, _ = select.select([sys.stdin], [], [], timeout)
#             if rlist:
#                 key = sys.stdin.read(1)
#             else:
#                 key = None
#         else:
#             key = sys.stdin.read(1)
#     finally:
#         termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
#     return key

# class KeyInputNode(Node):
#     def __init__(self):
#         super().__init__("key_input_node")
#         self.get_logger().info("Key Input Node Initialized")
#         self.timer = self.create_timer(0.1, self.timer_callback)  
#         self.publisher = self.create_publisher(String, 'key_input', 10)
#         self.last_key_time = 0
    
#     def timer_callback(self):
#         key = self.get_key()
#         msg = String()
#         if key == None:
#             msg.data = ""
#         elif key:
#             msg.data = key
        
#         self.publisher.publish(msg)
#         if key == 'q':
#             raise SystemExit
    
#     def get_key(self):
#         key = key_listener(timeout=0.1)
#         current_time = time.time()
        
#         if key:
#             self.last_key_time = current_time
#             return key
#         elif current_time - self.last_key_time >= DEBOUNCE_DELAY:
#             return None
        
# def main(args=None):
#     rclpy.init(args=args)
#     node = KeyInputNode()    
#     try:
#         rclpy.spin(node)
#     except SystemExit:
#         rclpy.logging.get_logger("Quitting").info('Done')

#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import sys
import termios
import tty
import select
import time
from geometry_msgs.msg import Twist

DEBOUNCE_DELAY = 0.5

def key_listener(timeout=0):
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        if timeout:
            rlist, _, _ = select.select([sys.stdin], [], [], timeout)
            if rlist:
                key = sys.stdin.read(1)
            else:
                key = None
        else:
            key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

class KeyInputNode(Node):
    def __init__(self):
        super().__init__("key_input_node")
        self.get_logger().info("Key Input Node Initialized")
        self.timer = self.create_timer(0.1, self.timer_callback)  
        self.publisher = self.create_publisher(Twist, 'cmd_vel_joy', 10)
        self.last_key_time = 0
    
    def timer_callback(self):
        key = self.get_key()
        msg = Twist()
        if key:
            if key == 'w':
                msg.linear.x = 1.0
            elif key == 's':
                msg.linear.x = -1.0
            elif key == 'a':
                msg.angular.z = 1.0
            elif key == 'd':
                msg.angular.z = -1.0
            elif key == 'q':
                raise SystemExit
        
        self.publisher.publish(msg)
    
    def get_key(self):
        key = key_listener(timeout=0.1)
        current_time = time.time()
        
        if key:
            self.last_key_time = current_time
            return key
        elif current_time - self.last_key_time >= DEBOUNCE_DELAY:
            return None
        
def main(args=None):
    rclpy.init(args=args)
    node = KeyInputNode()    
    try:
        rclpy.spin(node)
    except SystemExit:
        rclpy.logging.get_logger("Quitting").info('Done')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()