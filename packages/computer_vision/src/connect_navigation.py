#!/usr/bin/env python3

import rospy
from duckietown.dtros import DTROS, NodeType
import importlib
import sys
import os

class ConnectNavigation(DTROS):
    def __init__(self, node_name):
        super(ConnectNavigation, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        
        rospy.loginfo("Initializing connection between navigation and color line handler")
        
        # Import the navigation_control module
        try:
            # Get the path to the src directory
            src_path = os.path.join(os.path.dirname(os.path.realpath(__file__)))
            sys.path.append(src_path)
            
            # Import modules
            from navigation_control import NavigationControl
            from color_line_handler import ColorLineHandler
            
            # Get node handles
            rospy.sleep(2.0)  # Wait for nodes to fully initialize
            
            # Create a new navigation control instance
            self.nav = NavigationControl(node_name='navigation_proxy')
            
            # Find the color_line_handler instance
            color_handler_nodes = [node for node in rospy.get_node_names() 
                                  if 'color_line_handler_node' in node]
            
            if color_handler_nodes:
                color_handler_node = color_handler_nodes[0]
                rospy.loginfo(f"Found color line handler: {color_handler_node}")
                
                # Get the instance of ColorLineHandler
                # This is a bit of a hack - in a real system, you'd use services or actions
                # But for the purposes of this exercise, we're directly connecting the classes
                for node in rospy.core.get_global_node_names():
                    if hasattr(node, '__class__') and node.__class__.__name__ == 'ColorLineHandler':
                        color_handler = node
                        color_handler.set_navigator(self.nav)
                        rospy.loginfo("Connected navigation to color line handler")
                        break
                else:
                    rospy.logwarn("Could not find ColorLineHandler instance")
            else:
                rospy.logwarn("Color line handler node not found")
                
        except ImportError as e:
            rospy.logerr(f"Could not import navigation or color line handler modules: {e}")
        except Exception as e:
            rospy.logerr(f"Error connecting navigation to color line handler: {e}")
            
        rospy.loginfo("Connection setup complete")

if __name__ == '__main__':
    rospy.init_node('connect_navigation_node')
    node = ConnectNavigation(node_name='connect_navigation_node')
    
    # Keep the node running
    rospy.spin()