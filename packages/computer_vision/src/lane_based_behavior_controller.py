#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from enum import Enum

class BehaviorState(Enum):
    SEARCHING = 0
    APPROACHING = 1
    STOPPED = 2
    EXECUTING = 3
    COMPLETED = 4

class LaneBasedBehaviorController(DTROS):
    def __init__(self, node_name):
        super(LaneBasedBehaviorController, self).__init__(node_name=node_name, node_type=NodeType.CONTROL)
        
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
        
        # Subscribing to original images for detection
        self.sub_undistorted = rospy.Subscriber("lane_detection_node/image_undistorted/compressed",
                                              CompressedImage,
                                              self.image_callback,
                                              queue_size=1)
        
        # Current state tracking
        self.current_state = BehaviorState.SEARCHING
        self.current_color = None
        self.target_color = None
        self.behavior_start_time = None
        self.last_color_time = None
        
        # Detection parameters
        self.color_detected = {
            "blue": False,
            "red": False,
            "green": False
        }
        
        self.color_params = {
            "blue": None,
            "red": None,
            "green": None
        }
        
        # Navigation services
        self.nav_service_blue = rospy.ServiceProxy("handle_blue_line", String)
        self.nav_service_red = rospy.ServiceProxy("handle_red_line", String)
        self.nav_service_green = rospy.ServiceProxy("handle_green_line", String)
        
        # Initialize timer for behavior execution
        self.timer = rospy.Timer(rospy.Duration(0.1), self.execute_behavior)
        
        rospy.loginfo(f"{node_name} initialized")
    
    def image_callback(self, msg):
        """Process undistorted images for lane detection"""
        try:
            # Process the undistorted image if needed
            pass
        except Exception as e:
            rospy.logerr(f"Error in image callback: {e}")
    
    def blue_callback(self, msg):
        """Process blue line detection results"""
        try:
            # Convert compressed image to CV2
            np_arr = np.frombuffer(msg.data, np.uint8)
            blue_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            # Mark blue as detected
            self.color_detected["blue"] = True
            self.last_color_time = rospy.Time.now()
            
            # Extract center and dimensions
            # This would require additional processing of the image
            # For simplicity, we'll assume the detection node already computed these
            # and they could be passed via a custom message type
            
            # For demo, let's just create dummy values
            self.color_params["blue"] = (320, 400, 50, 20)  # x, y, width, height
            
            if self.current_state == BehaviorState.SEARCHING:
                self.target_color = "blue"
                self.current_state = BehaviorState.APPROACHING
                rospy.loginfo("Blue line detected, approaching")
                
        except Exception as e:
            rospy.logerr(f"Error in blue callback: {e}")
    
    def red_callback(self, msg):
        """Process red line detection results"""
        try:
            # Convert compressed image to CV2
            np_arr = np.frombuffer(msg.data, np.uint8)
            red_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            # Mark red as detected
            self.color_detected["red"] = True
            self.last_color_time = rospy.Time.now()
            
            # For demo, let's just create dummy values
            self.color_params["red"] = (320, 400, 50, 20)  # x, y, width, height
            
            if self.current_state == BehaviorState.SEARCHING:
                self.target_color = "red"
                self.current_state = BehaviorState.APPROACHING
                rospy.loginfo("Red line detected, approaching")
                
        except Exception as e:
            rospy.logerr(f"Error in red callback: {e}")
    
    def green_callback(self, msg):
        """Process green line detection results"""
        try:
            # Convert compressed image to CV2
            np_arr = np.frombuffer(msg.data, np.uint8)
            green_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            # Mark green as detected
            self.color_detected["green"] = True
            self.last_color_time = rospy.Time.now()
            
            # For demo, let's just create dummy values
            self.color_params["green"] = (320, 400, 50, 20)  # x, y, width, height
            
            if self.current_state == BehaviorState.SEARCHING:
                self.target_color = "green"
                self.current_state = BehaviorState.APPROACHING
                rospy.loginfo("Green line detected, approaching")
                
        except Exception as e:
            rospy.logerr(f"Error in green callback: {e}")
    
    def execute_behavior(self, event):
        """Main behavior execution state machine"""
        
        # Reset color detection after a timeout
        if self.last_color_time is not None:
            time_since_detection = (rospy.Time.now() - self.last_color_time).to_sec()
            if time_since_detection > 0.5:  # Reset after 0.5 seconds
                self.color_detected = {
                    "blue": False,
                    "red": False,
                    "green": False
                }
        
        if self.current_state == BehaviorState.SEARCHING:
            # Looking for colored lines
            # This state mainly relies on the callbacks to transition
            pass
            
        elif self.current_state == BehaviorState.APPROACHING:
            # Approaching the detected line
            if self.target_color == "blue":
                # Call navigation service to approach blue line
                # This should be implemented by calling the navigation node
                try:
                    # self.nav_service_blue("approach")
                    rospy.loginfo("Approaching blue line")
                    
                    # For demo, directly transition to STOPPED state
                    self.current_state = BehaviorState.STOPPED
                    self.behavior_start_time = rospy.Time.now()
                except Exception as e:
                    rospy.logerr(f"Error calling blue line service: {e}")
                    
            elif self.target_color == "red":
                # Call navigation service to approach red line
                try:
                    # self.nav_service_red("approach")
                    rospy.loginfo("Approaching red line")
                    
                    # For demo, directly transition to STOPPED state
                    self.current_state = BehaviorState.STOPPED
                    self.behavior_start_time = rospy.Time.now()
                except Exception as e:
                    rospy.logerr(f"Error calling red line service: {e}")
                    
            elif self.target_color == "green":
                # Call navigation service to approach green line
                try:
                    # self.nav_service_green("approach")
                    rospy.loginfo("Approaching green line")
                    
                    # For demo, directly transition to STOPPED state
                    self.current_state = BehaviorState.STOPPED
                    self.behavior_start_time = rospy.Time.now()
                except Exception as e:
                    rospy.logerr(f"Error calling green line service: {e}")
        
        elif self.current_state == BehaviorState.STOPPED:
            # Stopped at the line, waiting before executing the behavior
            time_stopped = (rospy.Time.now() - self.behavior_start_time).to_sec()
            
            if time_stopped >= 3.0:  # Wait for 3 seconds
                self.current_state = BehaviorState.EXECUTING
                self.behavior_start_time = rospy.Time.now()
                rospy.loginfo(f"Executing behavior for {self.target_color} line")
        
        elif self.current_state == BehaviorState.EXECUTING:
            # Execute behavior based on color
            if self.target_color == "blue":
                # Signal right and turn right
                try:
                    # self.nav_service_blue("execute")
                    rospy.loginfo("Signaling right and turning right")
                    
                    # For demo, directly transition to COMPLETED state
                    self.current_state = BehaviorState.COMPLETED
                except Exception as e:
                    rospy.logerr(f"Error calling blue line service: {e}")
                    
            elif self.target_color == "red":
                # Move straight
                try:
                    # self.nav_service_red("execute")
                    rospy.loginfo("Moving straight")
                    
                    # For demo, directly transition to COMPLETED state
                    self.current_state = BehaviorState.COMPLETED
                except Exception as e:
                    rospy.logerr(f"Error calling red line service: {e}")
                    
            elif self.target_color == "green":
                # Signal left and turn left
                try:
                    # self.nav_service_green("execute")
                    rospy.loginfo("Signaling left and turning left")
                    
                    # For demo, directly transition to COMPLETED state
                    self.current_state = BehaviorState.COMPLETED
                except Exception as e:
                    rospy.logerr(f"Error calling green line service: {e}")
        
        elif self.current_state == BehaviorState.COMPLETED:
            # Reset state and look for new lines
            self.current_state = BehaviorState.SEARCHING
            self.target_color = None
            rospy.loginfo("Behavior completed, searching for new lines")

if __name__ == '__main__':
    rospy.init_node('lane_based_behavior_controller_node')
    node = LaneBasedBehaviorController(node_name='lane_based_behavior_controller_node')
    rospy.spin()