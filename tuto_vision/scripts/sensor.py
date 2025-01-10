#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
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
        self.color_topic = "/sensor_msgs/color/image_raw"
        
    def initializeROSNode(self, aROSNode):
        # Création du timer pour appeler la méthode de callback
        self.timer = aROSNode.create_timer(0.033, self.timer_callback)  # 30Hz
        
        # Initialisation de la caméra
        self._init_camera()

    def _init_camera(self):
        # Configuration de la caméra pour la couleur
        self.config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, self.fps)
        
        # Démarrage du pipeline
        self.pipeline.start(self.config)
        print("Camera initialized.")

    def timer_callback(self):
        # Fonction qui sera appelée périodiquement par le timer
        color_image, depth_image = self.read_imgs()
        
        if color_image is not None:
            self.detect_object(color_image)

    def read_imgs(self):
        # Attente de frames de couleur
        frames = self.pipeline.wait_for_frames()
        self.color_frame = frames.get_color_frame()
        
        if not self.color_frame:
            return None, None
        
        # Conversion en tableaux numpy
        self.color_image = np.asanyarray(self.color_frame.get_data())
        return self.color_image, None

    def detect_object(self, color_image):
        # Application d'un masque basé sur la couleur pour détecter un objet
        # Exemple de seuil de couleur pour détecter un objet (ici, un objet rouge)
        hsv_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
        lower_red = np.array([0, 120, 70])
        upper_red = np.array([10, 255, 255])
        
        mask = cv2.inRange(hsv_image, lower_red, upper_red)
        white_pixels = cv2.countNonZero(mask)  # Nombre de pixels blancs (objets détectés)

        if white_pixels > 100:  # Si un nombre suffisant de pixels sont détectés
            print(f"Objet détecté avec {white_pixels} pixels.")

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
    
    # Initialize ROS node and camera
    camera.initializeROSNode(rosNode)
    
    # Start infinite loop
    while isOk:
        rclpy.spin_once(rosNode, timeout_sec=0.001)
    
    # Clean end
    camera.disconnect()
    rosNode.destroy_node()
    rclpy.shutdown()

# Entry point for the ROS node
if __name__ == '__main__':
    mainFunction()
