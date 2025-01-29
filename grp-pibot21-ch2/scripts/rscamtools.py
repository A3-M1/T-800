#!/usr/bin/python3

# Our own module to make the connection with the realsense camera easier, speed up programming on have more readable code

import pyrealsense2 as rs
import numpy as np


class RealSenseCam:
    def __init__(self):
        pipeline, config, align, colorizer = None, None, None, None


    def initRealsenseCamera(self, width=640, height=480, fps=30):
        """
        Initializes the RealSense camera pipeline with the desired configuration.
        """
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.colorizer = rs.colorizer()
        self.config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        self.config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        self.pipeline.start(self.config)
        align_to = rs.stream.color
        self.align = rs.align(align_to)

    def closeRealsenseCamera(self):
        """
        Stops the Realsense camera pipeline.
        """
        self.pipeline.stop()


    def getRealsenseFrames(self):
        """
        Get depth and color frames from a RealSense pipeline.

        Returns:
            A tuple containing the depth frame and color frame.
        """
        # Wait for frames and get them
        frames = self.pipeline.wait_for_frames()
        # Return frames
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        return color_frame, depth_frame
    
    def getRealsenseAlignedOnColorFrames(self):
        """
        Get depth and color frames from a RealSense pipeline.
        The color frame is here aligned with the depth frame.

        Returns:
            A tuple containing the aligned depth frame and color frame.
        """
        # Wait for frames and get them
        frames = self.pipeline.wait_for_frames()
        #Aligning color frame to depth frame
        aligned_frames = self.align.process(frames)
        # Return aligned frames
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        return color_frame, depth_frame

    def colorizeDepthFrame(self, depth_frame):
        """
        Colorizes a depth frame using the RealSense colorizer.

        Args:
            depth_frame: The depth frame to be colorized.

        Returns:
            The colorized depth frame as a numpy array.
        """
        # Use the colorizer to colorize the depth frame
        return self.colorizer.colorize(depth_frame)
    
    def convertToNumpyArray(self, frame):
        """
        Converts a RealSense frame to a numpy array.

        Args:
            frame: The frame to be converted.

        Returns:
            The frame as a numpy array.
        """
        return np.asanyarray(frame.get_data())
    
    def getRealPositionOfPixel(self, color_frame, depth_frame, x, y):
        """
        Get the position in real life relative to the camera base of a pixel in the depth frame.

        Args:
            depth_frame: The depth frame.
            x: The x-coordinate of the point.
            y: The y-coordinate of the point.

        Returns:
            A tuple containing the x, y and z coordinates of the point relative to the camera.
        """
        x, y = int(x), int(y)
        # Get the intrinsic parameters
        color_intrin = color_frame.profile.as_video_stream_profile().intrinsics
        # Calculate and return the coordinate
        depth = depth_frame.get_distance(x, y)
        return rs.rs2_deproject_pixel_to_point(color_intrin, [x,y], depth)
    
    """
    def estimatePositionOfElement(self, color_frame, depth_frame, x, y, radius=6):
        
        Estimate the position of an element in real life relative to the camera base.
        This do the mean of the position of the pixels in the square of the given radius around the center of the element.

        Args:
            color_frame: The color frame.
            depth_frame: The depth frame.
            x: The x-coordinate of the point.
            y: The y-coordinate of the point.
            radius: The radius of the element. Defaults to 6.

        Returns:
            A tuple containing the x, y and z coordinates of the element relative to the camera.
        
        x, y = int(x), int(y)
        distances = np.array([self.getRealPositionOfPixel(color_frame, depth_frame, i, j) for j in range(x-radius, x+radius) for i in range(y-radius, y+radius)])
        distances[distances == 0] = np.nan
        mean = np.nanmean(distances, axis=0)
        return mean if not np.any(np.isnan(mean)) else (0, 0, 0)
    """
        
        