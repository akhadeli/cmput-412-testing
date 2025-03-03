#!/usr/bin/env python3

import rospy
import argparse
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String, Bool
from nav_msgs.msg import Odometry

class LaneFollowing(DTROS):
    def __init__(self, node_name):
        super(LaneFollowing, self).__init__(node_name=node_name, node_type=NodeType.CONTROL)
        
        # Command publisher to controller
        self.pub_lane_control_cmd = rospy.Publisher("lane_following_controller_node/controller_cmd", String, queue_size=1)
        
        # Enable/disable lane following
        self.pub_lane_control_enable = rospy.Publisher("lane_following_controller_node/enable", Bool, queue_size=1)
        
        # Subscribe to keyboard commands for debugging
        self.sub_keyboard = rospy.Subscriber("~commands", String, self.command_callback, queue_size=1)
        
        # Initialize state
        self.is_following_lane = False
        self.current_controller = "P"
        
        # Parse arguments
        self.args = self.parse_args()
        self.process_args()
        
        rospy.loginfo(f"{node_name} initialized")
    
    def parse_args(self):
        """Parse command line arguments"""
        parser = argparse.ArgumentParser(description='Lane Following Node')
        parser.add_argument('--controller', type=str, default='P',
                           choices=['P', 'PD', 'PID'],
                           help='Controller type to use')
        parser.add_argument('--auto_start', type=bool, default=False,
                           help='Automatically start lane following')
        
        return parser.parse_args()
    
    def process_args(self):
        """Process command line arguments"""
        if self.args.controller not in ['P', 'PD', 'PID']:
            rospy.logwarn(f"Unknown controller type: {self.args.controller}, using P controller")
            self.current_controller = "P"
        else:
            self.current_controller = self.args.controller
            
        if self.args.auto_start:
            rospy.sleep(2.0)  # Wait for other nodes to initialize
            self.start_lane_following()
    
    def command_callback(self, msg):
        """Process commands from keyboard or other sources"""
        command = msg.data.strip().lower()
        
        if command == "start":
            self.start_lane_following()
        elif command == "stop":
            self.stop_lane_following()
        elif command == "p":
            self.set_controller("P")
        elif command == "pd":
            self.set_controller("PD")
        elif command == "pid":
            self.set_controller("PID")
        else:
            rospy.logwarn(f"Unknown command: {command}")
    
    def start_lane_following(self):
        """Start lane following"""
        if not self.is_following_lane:
            rospy.loginfo("Starting lane following")
            self.is_following_lane = True
            
            # Set controller type
            self.pub_lane_control_cmd.publish(self.current_controller)
            
            # Enable lane following
            self.pub_lane_control_enable.publish(True)
    
    def stop_lane_following(self):
        """Stop lane following"""
        if self.is_following_lane:
            rospy.loginfo("Stopping lane following")
            self.is_following_lane = False
            
            # Disable lane following
            self.pub_lane_control_enable.publish(False)
    
    def set_controller(self, controller_type):
        """Set the controller type"""
        if controller_type in ['P', 'PD', 'PID']:
            rospy.loginfo(f"Setting controller to {controller_type}")
            self.current_controller = controller_type
            
            # Send controller type to lane following node
            self.pub_lane_control_cmd.publish(controller_type)
        else:
            rospy.logwarn(f"Unknown controller type: {controller_type}")

def main():
    rospy.init_node('lane_following_node')
    node = LaneFollowing(node_name='lane_following_node')
    
    # Set up a run loop at 10 Hz
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        # Any periodic tasks can go here
        rate.sleep()

if __name__ == '__main__':
    main()