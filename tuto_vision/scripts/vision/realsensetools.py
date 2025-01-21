import pyrealsense2 as rs
import numpy as np


def initRealsenseCamera():
    """
    Initializes the RealSense camera pipeline with the desired configuration.

    Returns:
        pipeline (rs.pipeline()): The initialized RealSense camera pipeline.
    """
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)
    return pipeline, config

def closeRealsenseCamera(pipeline):
    """
    Stops the Realsense camera pipeline.

    Args:
        pipeline: The Realsense pipeline object.

    Returns:
        None
    """
    pipeline.stop()


def getRealsenseFrames(pipeline):
    """
    Get depth and color frames from a RealSense pipeline.

    Args:
        pipeline: The RealSense pipeline object.

    Returns:
        A tuple containing the depth frame and color frame.
    """
    frames = pipeline.wait_for_frames()
    depth_frame = np.asanyarray(frames.get_depth_frame().get_data())
    color_frame = np.asanyarray(frames.get_color_frame().get_data())
    return color_frame, depth_frame

def colorizeDepthFrame(depth_frame):
    """
    Colorizes a depth frame using the RealSense colorizer.

    Args:
        depth_frame: The depth frame to be colorized.

    Returns:
        The colorized depth frame as a numpy array.
    """
    colorizer = rs.colorizer()
    colorized_depth = np.asanyarray(colorizer.colorize(depth_frame).get_data())
    return colorized_depth