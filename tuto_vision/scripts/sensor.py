
#!/usr/bin/env python3

import pyrealsense2 as rs
import signal, time, numpy as np
import sys, cv2, rclpy

# Node processes:
def mainFunction(args=None):
    # Initialize ROS Node connected to the camera
    rclpy.init(args=args)
    rosNode= Node( "RealSense_driver" )
    camera= Realsense()
    camera.initializeROSNode( rosNode )

    # Start infinite loop
    while isOk:
        camera.read_imgs()
        camera.publish_imgs()
        rclpy.spin_once(rosNode, timeout_sec=0.001)

    # Clean end
    camera.disconnect()
    rosNode.destroy_node()
    rclpy.shutdown()

    # Realsense Node:
class Realsense():
    def __init__(self, fps= 60):
        # Initialize attributes
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        
        # Connect the camera
        
        
        pass

    def initializeROSNode( self, aROSNode ):
        # Initialize publishers / subscribers and timers
        pass

    def read_imgs(self):
        # Read data from camera
        pass



    def publish_imgs(self):
        # Send data into ROS Topics
        pass

    def disconnect(self):
        # disconnect the camera, free the resource.
        pass