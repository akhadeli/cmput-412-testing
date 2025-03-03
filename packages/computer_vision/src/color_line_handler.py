#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String, Bool
from cv_bridge import CvBridge, CvBridgeError
from duckietown_msgs.srv import SetCustomLEDPattern
from duckietown_msgs.msg import LEDPattern
import time

class ColorLineHandler(DTROS):
    def __init__(self, node_name):
        super(ColorLineHandler, self).__init__(node_name=node_name, node_type=NodeType.CONTROL)
        
        self.bridge = CvBridge()
        
        # Subscribe to color detection topics
        self.sub_blue = rospy.Subscriber("lane_detection_node/blue_detection/compressed", 
                                        CompressedImage, 
                                        self.blue_callback, 
                                        queue_size=1)
        
        self.sub_red = rospy.Subscriber("lane_detection_node/red_detection/compressed", 
                                       CompressedImage, 
                                       self.red_callback, 
                                       queue_size=1)
        
        self.sub_green = rospy.Subscriber("lane_detection_node/green_detection/compressed", 
                                         CompressedImage, 
                                         self.green_callback, 
                                         queue_size=1)
        
        # Color state tracking
        self.blue_detected = False
        self.red_detected = False
        self.green_detected = False
        
        self.blue_params = None  # x, y, width, height
        self.red_params = None
        self.green_params = None
        
        # Current state
        self.executing_behavior = False
        self.current_color = None
        
        # Create service for LED control
        self.led_service_name = "led_emitter_node/set_custom_pattern"
        rospy.wait_for_service(self.led_service_name)
        self.led_service = rospy.ServiceProxy(self.led_service_name, SetCustomLEDPattern)
        
        # Navigation object - we'll use direct function calls
        # rather than services for simplicity
        # In a real implementation, you might want to use ROS services or action servers
        self.navigator = None  # Will be initialized later
        
        # Command publisher - can be used to send commands to other nodes
        self.pub_command = rospy.Publisher("~command", String, queue_size=10)
        
        # Initialize timer for behavior execution
        self.timer = rospy.Timer(rospy.Duration(0.5), self.check_for_behaviors)
        
        rospy.loginfo(f"{node_name} initialized")
    
    def set_navigator(self, navigator):
        """Set the navigator object for executing movement commands"""
        self.navigator = navigator
        rospy.loginfo("Navigator set for color line handler")
    
    def blue_callback(self, msg):
        """Process blue line detection"""
        if self.executing_behavior:
            return
            
        try:
            # Convert compressed image to CV2
            np_arr = np.frombuffer(msg.data, np.uint8)
            blue_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            # In a real implementation, you would extract the position and size
            # of the blue line from the image or from custom message fields
            
            # For demo, we'll just use a placeholder
            if not self.blue_detected:
                rospy.loginfo("Blue line detected")
            
            self.blue_detected = True
            self.blue_params = (320, 400, 50, 20)  # x, y, width, height
        except Exception as e:
            rospy.logerr(f"Error in blue callback: {e}")
    
    def red_callback(self, msg):
        """Process red line detection"""
        if self.executing_behavior:
            return
            
        try:
            # Convert compressed image to CV2
            np_arr = np.frombuffer(msg.data, np.uint8)
            red_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            # For demo, we'll just use a placeholder
            if not self.red_detected:
                rospy.loginfo("Red line detected")
            
            self.red_detected = True
            self.red_params = (320, 400, 50, 20)  # x, y, width, height
        except Exception as e:
            rospy.logerr(f"Error in red callback: {e}")
    
    def green_callback(self, msg):
        """Process green line detection"""
        if self.executing_behavior:
            return
            
        try:
            # Convert compressed image to CV2
            np_arr = np.frombuffer(msg.data, np.uint8)
            green_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            # For demo, we'll just use a placeholder
            if not self.green_detected:
                rospy.loginfo("Green line detected")
            
            self.green_detected = True
            self.green_params = (320, 400, 50, 20)  # x, y, width, height
        except Exception as e:
            rospy.logerr(f"Error in green callback: {e}")
    
    def set_led_pattern(self, color, frequency=0.0):
        """
        Set the LED pattern of the robot
        
        Args:
            color: String "red", "green", "blue", "yellow", etc.
            frequency: Blinking frequency (0 for solid)
        """
        pattern = LEDPattern()
        
        if color == "red":
            pattern.rgb_vals = [1.0, 0.0, 0.0]
        elif color == "green":
            pattern.rgb_vals = [0.0, 1.0, 0.0]
        elif color == "blue":
            pattern.rgb_vals = [0.0, 0.0, 1.0]
        elif color == "yellow":
            pattern.rgb_vals = [1.0, 1.0, 0.0]
        elif color == "off":
            pattern.rgb_vals = [0.0, 0.0, 0.0]
        else:
            rospy.logwarn(f"Unknown color: {color}")
            return
        
        pattern.color_list = ['front_right', 'front_left', 'back_right', 'back_left']
        pattern.frequency = frequency
        pattern.frequency_mask = [frequency > 0, frequency > 0, frequency > 0, frequency > 0]
        
        try:
            self.led_service(pattern)
        except rospy.ServiceException as e:
            rospy.logerr(f"LED service call failed: {e}")
    
    def set_right_led(self, color, frequency=0.0):
        """Set only the right side LEDs"""
        pattern = LEDPattern()
        
        if color == "red":
            pattern.rgb_vals = [1.0, 0.0, 0.0]
        elif color == "green":
            pattern.rgb_vals = [0.0, 1.0, 0.0]
        elif color == "blue":
            pattern.rgb_vals = [0.0, 0.0, 1.0]
        elif color == "yellow":
            pattern.rgb_vals = [1.0, 1.0, 0.0]
        elif color == "off":
            pattern.rgb_vals = [0.0, 0.0, 0.0]
        else:
            rospy.logwarn(f"Unknown color: {color}")
            return
        
        pattern.color_list = ['front_right', 'back_right']
        pattern.frequency = frequency
        pattern.frequency_mask = [frequency > 0, frequency > 0]
        
        try:
            self.led_service(pattern)
        except rospy.ServiceException as e:
            rospy.logerr(f"LED service call failed: {e}")
    
    def set_left_led(self, color, frequency=0.0):
        """Set only the left side LEDs"""
        pattern = LEDPattern()
        
        if color == "red":
            pattern.rgb_vals = [1.0, 0.0, 0.0]
        elif color == "green":
            pattern.rgb_vals = [0.0, 1.0, 0.0]
        elif color == "blue":
            pattern.rgb_vals = [0.0, 0.0, 1.0]
        elif color == "yellow":
            pattern.rgb_vals = [1.0, 1.0, 0.0]
        elif color == "off":
            pattern.rgb_vals = [0.0, 0.0, 0.0]
        else:
            rospy.logwarn(f"Unknown color: {color}")
            return
        
        pattern.color_list = ['front_left', 'back_left']
        pattern.frequency = frequency
        pattern.frequency_mask = [frequency > 0, frequency > 0]
        
        try:
            self.led_service(pattern)
        except rospy.ServiceException as e:
            rospy.logerr(f"LED service call failed: {e}")
    
    def handle_blue_line(self):
        """Handle blue line behavior: stop, signal right, turn right"""
        if self.navigator is None:
            rospy.logerr("Navigator not set, cannot execute behavior")
            return False
            
        try:
            self.executing_behavior = True
            self.current_color = "blue"
            
            # 1. Stop for 3-5 seconds
            rospy.loginfo("Blue line: Stopping for 3 seconds")
            self.navigator.stop(3.0)
            
            # 2. Signal on right side LEDs
            rospy.loginfo("Blue line: Signaling right")
            self.set_right_led("blue")
            
            # 3. Turn right 90 degrees
            rospy.loginfo("Blue line: Turning right")
            self.navigator.turn_right(radius=0.2)  # Using a small radius for smoother turn
            
            # Turn off LEDs when done
            self.set_led_pattern("off")
            
            rospy.loginfo("Blue line behavior completed")
            self.executing_behavior = False
            self.blue_detected = False
            return True
        except Exception as e:
            rospy.logerr(f"Error executing blue line behavior: {e}")
            self.executing_behavior = False
            return False
    
    def handle_red_line(self):
        """Handle red line behavior: stop, move straight"""
        if self.navigator is None:
            rospy.logerr("Navigator not set, cannot execute behavior")
            return False
            
        try:
            self.executing_behavior = True
            self.current_color = "red"
            
            # 1. Stop for 3-5 seconds
            rospy.loginfo("Red line: Stopping for 3 seconds")
            self.navigator.stop(3.0)
            
            # 2. Move straight for at least 30 cm
            rospy.loginfo("Red line: Moving straight")
            self.navigator.move_straight(0.3)  # 30 cm
            
            rospy.loginfo("Red line behavior completed")
            self.executing_behavior = False
            self.red_detected = False
            return True
        except Exception as e:
            rospy.logerr(f"Error executing red line behavior: {e}")
            self.executing_behavior = False
            return False
    
    def handle_green_line(self):
        """Handle green line behavior: stop, signal left, turn left"""
        if self.navigator is None:
            rospy.logerr("Navigator not set, cannot execute behavior")
            return False
            
        try:
            self.executing_behavior = True
            self.current_color = "green"
            
            # 1. Stop for 3-5 seconds
            rospy.loginfo("Green line: Stopping for 3 seconds")
            self.navigator.stop(3.0)
            
            # 2. Signal on left side LEDs
            rospy.loginfo("Green line: Signaling left")
            self.set_left_led("green")
            
            # 3. Turn left 90 degrees
            rospy.loginfo("Green line: Turning left")
            self.navigator.turn_left(radius=0.2)  # Using a small radius for smoother turn
            
            # Turn off LEDs when done
            self.set_led_pattern("off")
            
            rospy.loginfo("Green line behavior completed")
            self.executing_behavior = False
            self.green_detected = False
            return True
        except Exception as e:
            rospy.logerr(f"Error executing green line behavior: {e}")
            self.executing_behavior = False
            return False
    
    def check_for_behaviors(self, event):
        """Check for detected lines and execute appropriate behaviors"""
        if self.executing_behavior:
            return
            
        if self.blue_detected:
            rospy.loginfo("Executing blue line behavior")
            self.handle_blue_line()
        elif self.red_detected:
            rospy.loginfo("Executing red line behavior")
            self.handle_red_line()
        elif self.green_detected:
            rospy.loginfo("Executing green line behavior")
            self.handle_green_line()

if __name__ == '__main__':
    rospy.init_node('color_line_handler_node')
    node = ColorLineHandler(node_name='color_line_handler_node')
    
    # Note: In a real implementation, you would need to pass the 
    # navigator object to the node, or use services/actions instead
    
    rospy.spin()