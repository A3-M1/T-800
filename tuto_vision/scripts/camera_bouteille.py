#!/usr/bin/env python3

import pyrealsense2 as rs
import signal, time, numpy as np
import sys, cv2, rclpy

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

print(f"Connect: {device_product_line}")
found_rgb = True
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True

if not found_rgb:
    print("Depth camera required !!!")
    exit(0)

config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 60)
# config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 60)

# Capture ctrl-c event
isOk = True
def signalInteruption(signum, frame):
    global isOk
    print("\nCtrl-c pressed")
    isOk = False

signal.signal(signal.SIGINT, signalInteruption)

# Start streaming
pipeline.start(config)

count = 1
refTime = time.process_time()
freq = 60

sys.stdout.write("-")

nb_pacman = 0

while isOk:
    # Wait for a coherent tuple of frames: depth, color and accel
    frames = pipeline.wait_for_frames()

    color_frame = frames.first(rs.stream.color)
    #depth_frame = frames.first(rs.stream.depth)

    if not (color_frame):
        continue

    # Convert images to numpy arrays
    color_image = np.asanyarray(color_frame.get_data())
    color_colormap_dim = color_image.shape

    # Perform color thresholding on the live camera feed
    b, g, r = cv2.split(color_image)

    mask = np.uint8((r > 15) & (r < 170) & (g > 110) & (g < 200) & (b > 10) & (b < 100))
    r = r * mask
    g = g * mask
    b = b * mask
    newRGBImage = cv2.merge((b, g, r))

    mask_RGB = cv2.merge((mask, mask, mask)) * 255

    # Convertir le masque HSV en niveaux de gris
    mask = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(mask, np.array([30, 100, 50]), np.array([70, 255, 255]))

    # Convertir le masque en 3 canaux pour la concaténation
    mask_hsv = cv2.merge((mask, mask, mask))

    # Combiner l'image originale avec le masque
    seuillage_couleur = np.concatenate((color_image, mask_hsv), axis=1)

    # Comptage des pixels blancs dans le masque
    white_pixels = cv2.countNonZero(mask)  # Compte les pixels non-nuls (blancs) dans le masque
    # print(f"Nombre de pixels blancs: {white_pixels}")


    if white_pixels > 1000 :
        nb_pacman+=1
        print(f"Y'A UN PACMAN : {nb_pacman}")
    # Affichage et sauvegarde
    cv2.imwrite("seuillage_couleur.jpg", seuillage_couleur)
    cv2.imshow("seuillage_couleur", seuillage_couleur)

    # Display RealSense images
    #images = np.hstack(color_image)
    cv2.waitKey(1)

    # Frequency:
    if count == 10:
        newTime = time.process_time()
        freq = 10 / (newTime - refTime)
        refTime = newTime
        count = 0
    count += 1

# Stop streaming
print("\nEnding...")
pipeline.stop()
