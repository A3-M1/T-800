#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import pyrealsense2 as rs
import numpy as np
import cv2
import time

# Global flag to handle Ctrl-C interruption
isOk = True

# Function to handle Ctrl-C interrupt
def signalInteruption(signum, frame):
    global isOk
    print("\nCtrl-c pressed")
    isOk = False

import signal
signal.signal(signal.SIGINT, signalInteruption)

class Realsense():
    def __init__(self, fps=60):
        # Initialisation des attributs de la caméra
        self.fps = fps
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.color_stream = None
        self.depth_stream = None
        self.depth_topic = "/sensor_msgs/depth/image_raw"
        self.color_topic = "/sensor_msgs/color/image_raw"
        
    def initializeROSNode(self, aROSNode):
        # Configuration des publishers ROS
        self.color_pub = aROSNode.create_publisher(Image, self.color_topic, 10)
        self.depth_pub = aROSNode.create_publisher(Image, self.depth_topic, 10)
        
        # Création du timer pour appeler la méthode de callback
        self.timer = aROSNode.create_timer(0.033, self.timer_callback)  # 30Hz
        
        # Initialisation de la caméra
        self._init_camera()

    def _init_camera(self):
        # Configuration de la caméra pour la couleur et la profondeur
        self.config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, self.fps)
        self.config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, self.fps)
        
        # Démarrage du pipeline
        self.pipeline.start(self.config)
        print("Camera initialized.")

    def timer_callback(self):
        # Fonction qui sera appelée périodiquement par le timer
        color_image, depth_image = self.read_imgs()
        
        if color_image is not None and depth_image is not None:
            self.publish_imgs()

    def read_imgs(self):
        # Attente de frames de couleur et de profondeur
        frames = self.pipeline.wait_for_frames()
        
        self.color_frame = frames.get_color_frame()
        self.depth_frame = frames.get_depth_frame()
        
        if not (self.color_frame and self.depth_frame):
            return None, None
        
        # Conversion en tableaux numpy
        self.color_image = np.asanyarray(self.color_frame.get_data())
        self.depth_image = np.asanyarray(self.depth_frame.get_data())
        
        # Application d'un colormap sur l'image de profondeur pour la visualisation
        self.depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(self.depth_image, alpha=0.03), cv2.COLORMAP_JET)
        
        return self.color_image, self.depth_colormap

    def publish_imgs(self):
        # Conversion des images en messages ROS et publication
        if self.color_image is not None and self.depth_colormap is not None:
            color_msg = self._convert_to_ros_img(self.color_image)
            depth_msg = self._convert_to_ros_img(self.depth_colormap)
            
            self.color_pub.publish(color_msg)
            self.depth_pub.publish(depth_msg)

    def _convert_to_ros_img(self, cv_img):
        # Conversion d'une image OpenCV en message Image de ROS
        msg = Image()
        msg.header.stamp = rclpy.time.Time().to_msg()
        msg.height = cv_img.shape[0]
        msg.width = cv_img.shape[1]
        msg.encoding = "bgr8"  # Encodage pour l'image couleur
        msg.data = np.array(cv_img).tobytes()
        return msg

    def disconnect(self):
        # Arrêt du streaming et déconnexion de la caméra
        print("Disconnecting the camera...")
        self.pipeline.stop()

        
# Main ROS Node function
def mainFunction(args=None):
    # Initialize ROS Node
    rclpy.init(args=args)
    rosNode = Node("RealSense_driver")
    
    # Create Realsense object
    camera = Realsense()
    
    # Initialize ROS publishers and camera
    camera.initializeROSNode(rosNode)
    
    # Start infinite loop
    while isOk:
        color_image, depth_image = camera.read_imgs()
        
        # Publish images if they are valid
        if color_image is not None and depth_image is not None:
            camera.publish_imgs()
        
        # Handle ROS spin for communication
        rclpy.spin_once(rosNode, timeout_sec=0.001)
    
    # Clean end
    camera.disconnect()
    rosNode.destroy_node()
    rclpy.shutdown()

# Entry point for the ROS node
if __name__ == '__main__':
    mainFunction()