import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy
import marshal
import types
import os
import sys
import tty
import termios
import threading
import cv2
import numpy as np
import time
from datetime import datetime
from interfaces_pkg.msg import MotionCommand

def getch():
    """Gets a single character from stdin"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

class DataCollector:
    def __init__(self, path="./Collected_Datasets", cam_num=0, max_steering=7, capture_interval=0.5):
        self.path = path
        self.cam_num = cam_num
        self.max_steering = max_steering
        self.capture_interval = capture_interval
        self.reset_values()
        
        # Initialize camera
        self.cap = cv2.VideoCapture(self.cam_num)
        if not self.cap.isOpened():
            raise RuntimeError(f"Failed to open camera {self.cam_num}")
            
        # Create save directory if it doesn't exist
        os.makedirs(self.path, exist_ok=True)
        
        # Start camera thread
        self.camera_thread = threading.Thread(target=self.camera_loop)
        self.camera_thread.daemon = True
        self.camera_thread.start()
        
        self.frame = None
        self.continuous_capture = False
        self.last_capture_time = 0

    def reset_values(self):
        self.steering = 0
        self.left_speed = 0
        self.right_speed = 0
        self.exit_flag = False

    def process_key(self, key):
        if key == 'q':
            self.exit_flag = True
        elif key == 'r':  # Reset all values
            self.reset_values()
            print("\nReset all values to zero. Continuing...\n")
        elif key == 'c':  # Toggle continuous capture
            self.continuous_capture = not self.continuous_capture
            status = "started" if self.continuous_capture else "stopped"
            print(f"\nContinuous capture {status}\n")
        elif key == 'a':  # Left
            self.steering = max(-self.max_steering, self.steering - 1)
        elif key == 'd':  # Right
            self.steering = min(self.max_steering, self.steering + 1)
        elif key == 'w':  # Speed up
            self.left_speed = min(255, self.left_speed + 5)
            self.right_speed = min(255, self.right_speed + 5)
        elif key == 's':  # Speed down
            self.left_speed = max(-255, self.left_speed - 5)
            self.right_speed = max(-255, self.right_speed - 5)
        elif key == ' ':  # Space for emergency stop
            self.steering = 0
            self.left_speed = 0
            self.right_speed = 0

    def camera_loop(self):
        # Create info window
        cv2.namedWindow('Control Info', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Control Info', 400, 100)
        
        while not self.exit_flag:
            ret, self.frame = self.cap.read()
            if not ret:
                continue
            
            if self.frame is not None:
                # Show original frame without text
                cv2.imshow('Camera Feed', self.frame)
                
                # Create and update info window
                info_img = np.zeros((100, 400, 3), dtype=np.uint8)
                text = f"Steering: {self.steering}"
                cv2.putText(info_img, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                           0.7, (0, 255, 0), 2)
                text = f"Speed L/R: {self.left_speed}/{self.right_speed}"
                cv2.putText(info_img, text, (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 
                           0.7, (0, 255, 0), 2)
                
                # Add capture status
                capture_status = "Recording" if self.continuous_capture else "Stopped"
                cv2.putText(info_img, capture_status, (300, 30), cv2.FONT_HERSHEY_SIMPLEX,
                           0.7, (0, 0, 255) if self.continuous_capture else (128, 128, 128), 2)
                
                cv2.imshow('Control Info', info_img)
                
            # Handle continuous capture
            current_time = time.time()
            if self.continuous_capture and (current_time - self.last_capture_time) >= self.capture_interval:
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = os.path.join(self.path, f"{timestamp}_{self.steering}.jpg")
                cv2.imwrite(filename, self.frame)
                print(f"\nImage saved: {filename}\n")
                self.last_capture_time = current_time
                
            cv2.waitKey(1)

    def get_control_values(self):
        return {
            'steering': self.steering,
            'left_speed': self.left_speed,
            'right_speed': self.right_speed
        }

    def cleanup(self):
        self.exit_flag = True
        if self.cap is not None:
            self.cap.release()
        cv2.destroyAllWindows()

class DataCollectionNode(Node):
    def __init__(self):
        super().__init__('data_collection_node')
        
        # Parameters
        self.DATA_PATH = "./Collected_Datasets"
        self.CAMERA_NUM = 0
        self.MAX_STEERING = 7
        self.CAPTURE_INTERVAL = 0.5  # Capture interval in seconds
        
        # QoS Profile
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=1
        )
        
        # Publisher
        self.publisher = self.create_publisher(
            MotionCommand,
            'topic_control_signal',
            self.qos_profile
        )
        
        # Initialize data collector
        self.data_collector = DataCollector(
            path=self.DATA_PATH,
            cam_num=self.CAMERA_NUM,
            max_steering=self.MAX_STEERING,
            capture_interval=self.CAPTURE_INTERVAL
        )
        
        # Create timer for regular publishing
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz
        
        # Start input thread
        self.input_thread = threading.Thread(target=self.input_loop)
        self.input_thread.daemon = True
        self.input_thread.start()

        # Print instructions
        print("\nControls:")
        print("W/S: Increase/Decrease speed (including reverse)")
        print("A/D: Turn left/right")
        print("Space: Emergency stop")
        print("C: Toggle continuous capture mode")
        print("R: Reset all values to zero")
        print("Q: Quit\n")
        
    def input_loop(self):
        while rclpy.ok():
            key = getch()
            self.data_collector.process_key(key)
            if self.data_collector.exit_flag:
                break
        
    def timer_callback(self):
        if self.data_collector.exit_flag:
            self.publish_stop_command()
            rclpy.shutdown()
            return
            
        # Get current control values
        control_values = self.data_collector.get_control_values()
        
        # Create and publish ROS2 message
        msg = MotionCommand()
        msg.steering = control_values['steering']
        msg.left_speed = control_values['left_speed']
        msg.right_speed = control_values['right_speed']
        self.publisher.publish(msg)
    
    def publish_stop_command(self):
        msg = MotionCommand()
        msg.steering = 0
        msg.left_speed = 0
        msg.right_speed = 0
        self.publisher.publish(msg)
        self.get_logger().info("Published stop command")
    
    def cleanup(self):
        self.data_collector.exit_flag = True
        if self.input_thread.is_alive():
            self.input_thread.join(timeout=1.0)
        self.publish_stop_command()
        self.data_collector.cleanup()

def main(args=None):
    rclpy.init(args=args)
    node = DataCollectionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()