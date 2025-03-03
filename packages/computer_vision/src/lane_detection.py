#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import yaml
import os
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge, CvBridgeError
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String
from duckietown_msgs.msg import LEDPattern
from duckietown_msgs.srv import SetCustomLEDPattern

class LaneDetectionNode(DTROS):
    def __init__(self, node_name):
        super(LaneDetectionNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        # Bridge for OpenCV conversion
        self.bridge = CvBridge()
        
        # Load camera calibration parameters from yaml file
        # Change this path to your calibration file
        calibration_file = os.path.expanduser('~/camera_intrinsic.yaml')
        if not os.path.isfile(calibration_file):
            rospy.logwarn(f"Calibration file not found at {calibration_file}")
            # Use default parameters as fallback
            self.camera_matrix = np.array([
                [305.5718893575089, 0, 303.0797142544728],
                [0, 308.8338858195428, 231.8845403702499],
                [0, 0, 1]
            ])
            self.distortion_coeff = np.array([-0.2, 0.0305, 0.0005, 0.0005, 0])
        else:
            with open(calibration_file, 'r') as file:
                calibration_data = yaml.safe_load(file)
                self.camera_matrix = np.array(calibration_data['camera_matrix']['data']).reshape((3, 3))
                self.distortion_coeff = np.array(calibration_data['distortion_coefficients']['data'])
            
        # Color detection parameters in HSV format
        # These are initial values, should be tuned based on actual images
        self.blue_lower = np.array([100, 50, 50])
        self.blue_upper = np.array([130, 255, 255])
        
        self.red_lower1 = np.array([0, 70, 50])
        self.red_upper1 = np.array([10, 255, 255])
        self.red_lower2 = np.array([170, 70, 50])
        self.red_upper2 = np.array([180, 255, 255])
        
        self.green_lower = np.array([40, 50, 50])
        self.green_upper = np.array([80, 255, 255])
        
        self.yellow_lower = np.array([20, 100, 100])
        self.yellow_upper = np.array([30, 255, 255])
        
        self.white_lower = np.array([0, 0, 200])
        self.white_upper = np.array([180, 30, 255])
        
        # Subscribe to camera feed
        self.sub_camera = rospy.Subscriber("~image_in", CompressedImage, self.callback, queue_size=1)
        
        # Publishers
        self.pub_image_undistorted = rospy.Publisher("~image_undistorted/compressed", CompressedImage, queue_size=1)
        self.pub_blue_detection = rospy.Publisher("~blue_detection/compressed", CompressedImage, queue_size=1)
        self.pub_red_detection = rospy.Publisher("~red_detection/compressed", CompressedImage, queue_size=1)
        self.pub_green_detection = rospy.Publisher("~green_detection/compressed", CompressedImage, queue_size=1)
        self.pub_yellow_lane = rospy.Publisher("~yellow_lane/compressed", CompressedImage, queue_size=1)
        self.pub_white_lane = rospy.Publisher("~white_lane/compressed", CompressedImage, queue_size=1)
        
        # LED control
        self.led_service_name = "led_emitter_node/set_custom_pattern"
        rospy.wait_for_service(self.led_service_name)
        self.led_service = rospy.ServiceProxy(self.led_service_name, SetCustomLEDPattern)
        
        # ROI vertices - adjust based on your camera setup
        self.roi_vertices = np.array([
            [0, 480],
            [640, 480],
            [640, 300],
            [0, 300]
        ], dtype=np.int32)
        
        # Perception rate - reduce load on CPU
        self.processing_rate = rospy.Rate(5)  # 5 Hz
        
        rospy.loginfo(f"{node_name} initialized successfully")

    def undistort_image(self, image):
        """
        Correct lens distortion using camera calibration parameters
        """
        h, w = image.shape[:2]
        # Calculate optimal camera matrix
        new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
            self.camera_matrix, self.distortion_coeff, (w, h), 0, (w, h)
        )
        
        # Undistort the image
        undistorted = cv2.undistort(
            image, self.camera_matrix, self.distortion_coeff, None, new_camera_matrix
        )
        
        # Crop the image based on ROI
        x, y, w, h = roi
        if roi[2] > 0 and roi[3] > 0:  # Check if ROI is valid
            undistorted = undistorted[y:y+h, x:x+w]
        
        return undistorted

    def preprocess_image(self, image):
        """
        Resize and smooth the image for better processing
        """
        # Resize image if needed
        # image = cv2.resize(image, (640, 480))
        
        # Apply Gaussian blur to reduce noise
        blurred = cv2.GaussianBlur(image, (5, 5), 0)
        
        return blurred
    
    def detect_lane_color(self, image, lower, upper, second_lower=None, second_upper=None):
        """
        Detect a specific color in the image using HSV color space
        """
        # Convert to HSV color space
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Create mask for the color
        mask = cv2.inRange(hsv, lower, upper)
        
        # For colors like red that wrap around the HSV color space
        if second_lower is not None and second_upper is not None:
            mask2 = cv2.inRange(hsv, second_lower, second_upper)
            mask = cv2.bitwise_or(mask, mask2)
        
        # Apply mask to original image
        result = cv2.bitwise_and(image, image, mask=mask)
        
        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # If contours found, process them
        largest_contour = None
        max_area = 0
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 200 and area > max_area:  # Filter small contours
                max_area = area
                largest_contour = contour
        
        if largest_contour is not None:
            # Get bounding rectangle
            x, y, w, h = cv2.boundingRect(largest_contour)
            cv2.rectangle(result, (x, y), (x + w, y + h), (0, 255, 0), 2)
            
            # Calculate center of contour
            M = cv2.moments(largest_contour)
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                cv2.circle(result, (cx, cy), 5, (255, 255, 255), -1)
                
                return result, (cx, cy, w, h), True
        
        return result, None, False
    
    def detect_lane(self, image, color_lower, color_upper, second_lower=None, second_upper=None):
        """
        Detect lane lines of specific color
        """
        # Same as detect_lane_color but specifically for lane detection
        return self.detect_lane_color(image, color_lower, color_upper, second_lower, second_upper)
    
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
    
    def callback(self, msg):
        """
        Process incoming camera images
        """
        try:
            # Convert compressed image to CV2
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            # Undistort image
            undistorted_image = self.undistort_image(cv_image)
            
            # Preprocess image
            processed_image = self.preprocess_image(undistorted_image)
            
            # Detect lanes for control purposes (yellow dotted and white solid)
            yellow_lane_img, yellow_lane_params, yellow_detected = self.detect_lane(
                processed_image, self.yellow_lower, self.yellow_upper
            )
            
            white_lane_img, white_lane_params, white_detected = self.detect_lane(
                processed_image, self.white_lower, self.white_upper
            )
            
            # Publish lane detection results
            if yellow_detected:
                yellow_msg = CompressedImage()
                yellow_msg.header.stamp = rospy.Time.now()
                yellow_msg.format = "jpeg"
                yellow_msg.data = np.array(cv2.imencode('.jpg', yellow_lane_img)[1]).tostring()
                self.pub_yellow_lane.publish(yellow_msg)
            
            if white_detected:
                white_msg = CompressedImage()
                white_msg.header.stamp = rospy.Time.now()
                white_msg.format = "jpeg"
                white_msg.data = np.array(cv2.imencode('.jpg', white_lane_img)[1]).tostring()
                self.pub_white_lane.publish(white_msg)
            
            # Detect colored lines (blue, red, green)
            blue_img, blue_params, blue_detected = self.detect_lane_color(
                processed_image, self.blue_lower, self.blue_upper
            )
            
            red_img, red_params, red_detected = self.detect_lane_color(
                processed_image, self.red_lower1, self.red_upper1,
                self.red_lower2, self.red_upper2
            )
            
            green_img, green_params, green_detected = self.detect_lane_color(
                processed_image, self.green_lower, self.green_upper
            )
            
            # Publish color detection results
            if blue_detected:
                blue_msg = CompressedImage()
                blue_msg.header.stamp = rospy.Time.now()
                blue_msg.format = "jpeg"
                blue_msg.data = np.array(cv2.imencode('.jpg', blue_img)[1]).tostring()
                self.pub_blue_detection.publish(blue_msg)
            
            if red_detected:
                red_msg = CompressedImage()
                red_msg.header.stamp = rospy.Time.now()
                red_msg.format = "jpeg"
                red_msg.data = np.array(cv2.imencode('.jpg', red_img)[1]).tostring()
                self.pub_red_detection.publish(red_msg)
            
            if green_detected:
                green_msg = CompressedImage()
                green_msg.header.stamp = rospy.Time.now()
                green_msg.format = "jpeg"
                green_msg.data = np.array(cv2.imencode('.jpg', green_img)[1]).tostring()
                self.pub_green_detection.publish(green_msg)
            
            # Publish undistorted image
            undistorted_msg = CompressedImage()
            undistorted_msg.header.stamp = rospy.Time.now()
            undistorted_msg.format = "jpeg"
            undistorted_msg.data = np.array(cv2.imencode('.jpg', undistorted_image)[1]).tostring()
            self.pub_image_undistorted.publish(undistorted_msg)
            
            self.processing_rate.sleep()
            
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge error: {e}")
        except Exception as e:
            rospy.logerr(f"Error in callback: {e}")

if __name__ == '__main__':
    node = LaneDetectionNode(node_name='lane_detection_node')
    rospy.spin()