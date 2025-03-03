#!/usr/bin/env python3

import rospy
import numpy as np
import time
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import Twist2DStamped, WheelsCmdStamped

class NavigationControl(DTROS):
    def __init__(self, node_name):
        super(NavigationControl, self).__init__(node_name=node_name, node_type=NodeType.CONTROL)
        
        # Publishers for wheel commands
        self.pub_wheels = rospy.Publisher("~wheels_cmd", WheelsCmdStamped, queue_size=1)
        self.pub_car_cmd = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1)
        
        # Robot parameters
        self.wheel_distance = 0.1 # distance between the wheels in meters
        self.wheel_radius = 0.0318 # radius of the wheel in meters
        
        # Constants for movement
        self.linear_velocity = 0.2  # m/s
        self.angular_velocity = 1.2  # rad/s
        
        rospy.loginfo(f"{node_name} initialized")
        
    def publish_velocity(self, v, omega):
        """
        Publish velocity command to the car
        
        Args:
            v: linear velocity (m/s)
            omega: angular velocity (rad/s)
        """
        car_cmd_msg = Twist2DStamped()
        car_cmd_msg.header.stamp = rospy.Time.now()
        car_cmd_msg.v = v
        car_cmd_msg.omega = omega
        
        self.pub_car_cmd.publish(car_cmd_msg)
        
        # Also compute and publish direct wheel commands
        left_wheel_vel = (v - omega * self.wheel_distance / 2) / self.wheel_radius
        right_wheel_vel = (v + omega * self.wheel_distance / 2) / self.wheel_radius
        
        wheels_cmd_msg = WheelsCmdStamped()
        wheels_cmd_msg.header.stamp = rospy.Time.now()
        wheels_cmd_msg.vel_left = left_wheel_vel
        wheels_cmd_msg.vel_right = right_wheel_vel
        
        self.pub_wheels.publish(wheels_cmd_msg)
        
    def stop(self, duration=1.0):
        """
        Stop the robot for a specified duration
        
        Args:
            duration: time to stop in seconds
        """
        self.publish_velocity(0, 0)
        rospy.loginfo(f"Stopping for {duration} seconds")
        rospy.sleep(duration)
        
    def move_straight(self, distance, velocity=None, blocking=True):
        """
        Move the robot in a straight line for a specified distance
        
        Args:
            distance: distance to travel in meters
            velocity: linear velocity (m/s), uses default if None
            blocking: whether to block until the movement is complete
        """
        if velocity is None:
            velocity = self.linear_velocity
            
        # Ensure positive velocity, handle direction with distance sign
        velocity = abs(velocity) * np.sign(distance)
        distance = abs(distance)
        
        # Calculate time needed to cover the distance
        time_to_move = distance / abs(velocity)
        
        rospy.loginfo(f"Moving straight for {distance} meters at {velocity} m/s")
        
        # Start moving
        self.publish_velocity(velocity, 0)
        
        if blocking:
            # Wait for the calculated time
            start_time = rospy.Time.now().to_sec()
            while (rospy.Time.now().to_sec() - start_time) < time_to_move:
                # Keep publishing the velocity to ensure continuous movement
                self.publish_velocity(velocity, 0)
                rospy.sleep(0.1)  # Small sleep to avoid CPU hogging
            
            # Stop the robot
            self.stop()
        
    def turn_right(self, radius=None, angle=90, blocking=True):
        """
        Turn the robot to the right by a specified angle
        
        Args:
            radius: turning radius in meters, None for in-place rotation
            angle: angle to turn in degrees
            blocking: whether to block until the movement is complete
        """
        # Convert angle from degrees to radians
        angle_rad = np.deg2rad(angle)
        
        if radius is None:
            # In-place rotation
            omega = -self.angular_velocity  # Negative for right turn
            v = 0.0
            
            # Calculate time needed for the turn
            time_to_turn = angle_rad / abs(omega)
        else:
            # Arc movement
            omega = -self.angular_velocity / 2  # Reduce angular velocity for smoother turn
            v = abs(omega) * radius  # Linear velocity for arc movement
            
            # Calculate time needed for the turn
            time_to_turn = angle_rad / abs(omega)
        
        rospy.loginfo(f"Turning right {angle} degrees")
        
        # Start turning
        self.publish_velocity(v, omega)
        
        if blocking:
            # Wait for the calculated time
            start_time = rospy.Time.now().to_sec()
            while (rospy.Time.now().to_sec() - start_time) < time_to_turn:
                # Keep publishing the velocity to ensure continuous movement
                self.publish_velocity(v, omega)
                rospy.sleep(0.1)  # Small sleep to avoid CPU hogging
            
            # Stop the robot
            self.stop()
        
    def turn_left(self, radius=None, angle=90, blocking=True):
        """
        Turn the robot to the left by a specified angle
        
        Args:
            radius: turning radius in meters, None for in-place rotation
            angle: angle to turn in degrees
            blocking: whether to block until the movement is complete
        """
        # Convert angle from degrees to radians
        angle_rad = np.deg2rad(angle)
        
        if radius is None:
            # In-place rotation
            omega = self.angular_velocity  # Positive for left turn
            v = 0.0
            
            # Calculate time needed for the turn
            time_to_turn = angle_rad / abs(omega)
        else:
            # Arc movement
            omega = self.angular_velocity / 2  # Reduce angular velocity for smoother turn
            v = abs(omega) * radius  # Linear velocity for arc movement
            
            # Calculate time needed for the turn
            time_to_turn = angle_rad / abs(omega)
        
        rospy.loginfo(f"Turning left {angle} degrees")
        
        # Start turning
        self.publish_velocity(v, omega)
        
        if blocking:
            # Wait for the calculated time
            start_time = rospy.Time.now().to_sec()
            while (rospy.Time.now().to_sec() - start_time) < time_to_turn:
                # Keep publishing the velocity to ensure continuous movement
                self.publish_velocity(v, omega)
                rospy.sleep(0.1)  # Small sleep to avoid CPU hogging
            
            # Stop the robot
            self.stop()
    
if __name__ == '__main__':
    rospy.init_node('navigation_control_node')
    node = NavigationControl(node_name='navigation_control_node')
    rospy.spin()