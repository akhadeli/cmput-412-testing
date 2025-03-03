#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import Twist2DStamped, WheelsCmdStamped
from cv_bridge import CvBridge, CvBridgeError
from enum import Enum

class ControllerType(Enum):
    P = 0
    PD = 1
    PID = 2

class LaneFollowingController(DTROS):
    def __init__(self, node_name):
        super(LaneFollowingController, self).__init__(node_name=node_name, node_type=NodeType.CONTROL)
        
        # Initialize bridge and subscribe to lane detections
        self.bridge = CvBridge()
        
        # Subscribe to yellow and white lane detections
        self.sub_yellow = rospy.Subscriber("lane_detection_node/yellow_lane/compressed", 
                                         CompressedImage, 
                                         self.yellow_callback, 
                                         queue_size=1)
        
        self.sub_white = rospy.Subscriber("lane_detection_node/white_lane/compressed", 
                                        CompressedImage, 
                                        self.white_callback, 
                                        queue_size=1)
        
        # Publishers
        self.pub_car_cmd = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1)
        self.pub_wheels_cmd = rospy.Publisher("~wheels_cmd", WheelsCmdStamped, queue_size=1)
        
        # Visualization publisher
        self.pub_lane_viz = rospy.Publisher("~lane_viz/compressed", CompressedImage, queue_size=1)
        
        # Lane detection state
        self.yellow_detected = False
        self.white_detected = False
        self.yellow_center = None  # (x, y) of center of detected yellow lane
        self.white_center = None   # (x, y) of center of detected white lane
        
        # Control parameters
        self.target_x = 320  # Target x position (center of image)
        self.max_error = 200 # Maximum error value (for normalization)
        
        # P controller parameters
        self.Kp = 0.01
        
        # PD controller parameters
        self.Kd = 0.005
        self.prev_error = 0
        self.prev_time = rospy.get_time()
        
        # PID controller parameters
        self.Ki = 0.0008
        self.error_integral = 0
        self.max_integral = 100  # To prevent integral windup
        
        # Moving average for error smoothing
        self.error_history = []
        self.error_history_maxlen = 5
        
        # Robot parameters
        self.wheel_distance = 0.1   # distance between the wheels
        self.wheel_radius = 0.0318  # radius of the wheels
        
        # Controller selection
        self.controller_type = ControllerType.P  # Default to P controller
        
        # Driving parameters
        self.max_speed = 0.2  # m/s
        self.min_speed = 0.05  # m/s
        self.max_omega = 4.0  # rad/s
        
        # Timer for control updates
        self.control_rate = 10  # Hz
        self.timer = rospy.Timer(rospy.Duration(1.0/self.control_rate), self.control_callback)
        
        # Initialize control target publishing
        self.target_pub_timer = rospy.Timer(rospy.Duration(0.5), self.publish_target)
        
        # Flags
        self.is_running = False
        
        rospy.loginfo(f"{node_name} initialized with {self.controller_type} controller")
    
    def set_controller_type(self, controller_type):
        """Set the controller type to use"""
        if controller_type == "P":
            self.controller_type = ControllerType.P
        elif controller_type == "PD":
            self.controller_type = ControllerType.PD
        elif controller_type == "PID":
            self.controller_type = ControllerType.PID
        else:
            rospy.logwarn(f"Unknown controller type: {controller_type}, using P controller")
            self.controller_type = ControllerType.P
            
        # Reset controller state
        self.prev_error = 0
        self.error_integral = 0
        self.error_history = []
        
        rospy.loginfo(f"Controller type set to {self.controller_type}")
    
    def start(self):
        """Start lane following"""
        self.is_running = True
        rospy.loginfo("Lane following started")
    
    def stop(self):
        """Stop lane following"""
        self.is_running = False
        self.publish_velocity(0, 0)
        rospy.loginfo("Lane following stopped")
    
    def yellow_callback(self, msg):
        """Process yellow lane detection"""
        try:
            # Convert compressed image to CV2
            np_arr = np.frombuffer(msg.data, np.uint8)
            yellow_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            # This is a simplified approach - in a real implementation,
            # you would extract the center position from the image or a custom message
            
            # Find contours in the image
            gray = cv2.cvtColor(yellow_img, cv2.COLOR_BGR2GRAY)
            _, thresh = cv2.threshold(gray, 10, 255, cv2.THRESH_BINARY)
            contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                M = cv2.moments(largest_contour)
                
                if M["m00"] > 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    
                    self.yellow_center = (cx, cy)
                    self.yellow_detected = True
                    return
            
            # If no contours found or moments calculation failed
            self.yellow_detected = False
            self.yellow_center = None
            
        except Exception as e:
            rospy.logerr(f"Error in yellow lane callback: {e}")
            self.yellow_detected = False
            self.yellow_center = None
    
    def white_callback(self, msg):
        """Process white lane detection"""
        try:
            # Convert compressed image to CV2
            np_arr = np.frombuffer(msg.data, np.uint8)
            white_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            # Find contours in the image
            gray = cv2.cvtColor(white_img, cv2.COLOR_BGR2GRAY)
            _, thresh = cv2.threshold(gray, 10, 255, cv2.THRESH_BINARY)
            contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                M = cv2.moments(largest_contour)
                
                if M["m00"] > 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    
                    self.white_center = (cx, cy)
                    self.white_detected = True
                    return
            
            # If no contours found or moments calculation failed
            self.white_detected = False
            self.white_center = None
            
        except Exception as e:
            rospy.logerr(f"Error in white lane callback: {e}")
            self.white_detected = False
            self.white_center = None
    
    def calculate_error(self):
        """Calculate lane centering error"""
        # If both lanes detected, center between them
        if self.yellow_detected and self.white_detected:
            center_x = (self.yellow_center[0] + self.white_center[0]) / 2
            error = self.target_x - center_x
        
        # If only yellow lane detected, maintain a fixed distance from it
        elif self.yellow_detected:
            # Offset from yellow lane (assuming it's on the left)
            offset = 100  # pixels
            error = self.target_x - (self.yellow_center[0] + offset)
        
        # If only white lane detected, maintain a fixed distance from it
        elif self.white_detected:
            # Offset from white lane (assuming it's on the right)
            offset = 100  # pixels
            error = self.target_x - (self.white_center[0] - offset)
        
        # No lanes detected, no error (maintain current direction)
        else:
            error = 0
        
        # Add to error history for smoothing
        self.error_history.append(error)
        if len(self.error_history) > self.error_history_maxlen:
            self.error_history.pop(0)
        
        # Return smoothed error (moving average)
        smoothed_error = sum(self.error_history) / len(self.error_history)
        
        # Normalize error to [-1, 1] range
        normalized_error = np.clip(smoothed_error / self.max_error, -1, 1)
        
        return normalized_error
    
    def p_controller(self, error):
        """Proportional controller"""
        return self.Kp * error
    
    def pd_controller(self, error):
        """Proportional-Derivative controller"""
        current_time = rospy.get_time()
        dt = current_time - self.prev_time
        
        if dt <= 0:
            dt = 0.1  # Default to reasonable value if dt is zero or negative
        
        # Calculate derivative (rate of change of error)
        error_derivative = (error - self.prev_error) / dt
        
        # Update previous values
        self.prev_error = error
        self.prev_time = current_time
        
        # P + D control
        return self.Kp * error + self.Kd * error_derivative
    
    def pid_controller(self, error):
        """Proportional-Integral-Derivative controller"""
        current_time = rospy.get_time()
        dt = current_time - self.prev_time
        
        if dt <= 0:
            dt = 0.1  # Default to reasonable value if dt is zero or negative
        
        # Calculate derivative (rate of change of error)
        error_derivative = (error - self.prev_error) / dt
        
        # Calculate integral (sum of error over time)
        self.error_integral += error * dt
        
        # Limit integral to prevent windup
        self.error_integral = np.clip(self.error_integral, -self.max_integral, self.max_integral)
        
        # Update previous values
        self.prev_error = error
        self.prev_time = current_time
        
        # P + I + D control
        return self.Kp * error + self.Ki * self.error_integral + self.Kd * error_derivative
    
    def publish_velocity(self, v, omega):
        """Publish velocity commands"""
        car_cmd_msg = Twist2DStamped()
        car_cmd_msg.header.stamp = rospy.Time.now()
        car_cmd_msg.v = v
        car_cmd_msg.omega = omega
        
        self.pub_car_cmd.publish(car_cmd_msg)
        
        # Also calculate wheel commands
        left_wheel_vel = (v - omega * self.wheel_distance / 2) / self.wheel_radius
        right_wheel_vel = (v + omega * self.wheel_distance / 2) / self.wheel_radius
        
        wheels_cmd_msg = WheelsCmdStamped()
        wheels_cmd_msg.header.stamp = rospy.Time.now()
        wheels_cmd_msg.vel_left = left_wheel_vel
        wheels_cmd_msg.vel_right = right_wheel_vel
        
        self.pub_wheels_cmd.publish(wheels_cmd_msg)
    
    def control_callback(self, event):
        """Main control loop"""
        if not self.is_running:
            return
            
        try:
            # Calculate error
            error = self.calculate_error()
            
            # Select and apply controller based on type
            if self.controller_type == ControllerType.P:
                omega = self.p_controller(error)
            elif self.controller_type == ControllerType.PD:
                omega = self.pd_controller(error)
            elif self.controller_type == ControllerType.PID:
                omega = self.pid_controller(error)
            else:
                omega = self.p_controller(error)  # Default to P controller
            
            # Limit angular velocity
            omega = np.clip(omega, -self.max_omega, self.max_omega)
            
            # Adjust linear velocity based on angular velocity
            # Slow down when turning sharply for stability
            v = self.max_speed * (1 - 0.5 * abs(omega / self.max_omega))
            v = max(v, self.min_speed)  # Ensure minimum speed
            
            # Publish commands
            self.publish_velocity(v, omega)
            
        except Exception as e:
            rospy.logerr(f"Error in control callback: {e}")
    
    def publish_target(self, event):
        """Publish visualization of target and detected lanes"""
        if not (self.yellow_detected or self.white_detected):
            return
            
        # Create a blank image for visualization
        viz_img = np.zeros((480, 640, 3), dtype=np.uint8)
        
        # Draw the target line
        cv2.line(viz_img, (self.target_x, 0), (self.target_x, 480), (0, 255, 255), 2)
        
        # Draw the detected lane centers
        if self.yellow_detected:
            cv2.circle(viz_img, self.yellow_center, 10, (0, 255, 255), -1)
        
        if self.white_detected:
            cv2.circle(viz_img, self.white_center, 10, (255, 255, 255), -1)
        
        # Draw the current error
        error = self.calculate_error()
        cv2.putText(viz_img, f"Error: {error:.3f}", (20, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        
        # Draw the controller type
        cv2.putText(viz_img, f"Controller: {self.controller_type.name}", (20, 70),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        
        # Publish visualization
        viz_msg = CompressedImage()
        viz_msg.header.stamp = rospy.Time.now()
        viz_msg.format = "jpeg"
        viz_msg.data = np.array(cv2.imencode('.jpg', viz_img)[1]).tostring()
        self.pub_lane_viz.publish(viz_msg)

if __name__ == '__main__':
    rospy.init_node('lane_following_controller_node')
    node = LaneFollowingController(node_name='lane_following_controller_node')
    
    # By default, start with P controller and don't start lane following
    # until explicitly commanded
    
    rospy.spin()