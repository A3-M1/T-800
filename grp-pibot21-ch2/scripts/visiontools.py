#!/usr/bin/python3

# Our own module to make common vision tasks easier, speed up programming and have more readable code

import numpy as np
import cv2

def createHsvMask(frame, lo, hi, morphiterations=4, bluring=7):
    """
    Create a mask based on the HSV color space for a given frame.

    Args:
        frame (numpy.ndarray): The input frame.
        lo (tuple): The lower HSV threshold values.
        hi (tuple): The upper HSV threshold values.
        morphiterations (int, optional): The number of morphological iterations for erosion and dilation. Defaults to 4.
        bluring (int, optional): The size of the blurring kernel. Defaults to 7.

    Returns:
        numpy.ndarray: The generated mask.
    """
    # Convert lo and hi to numpy arrays
    lo = np.array(lo)
    hi = np.array(hi)
    # Convert the image to HSV and create a mask
    image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # Blur the image to help with shapes
    image = cv2.blur(image, (bluring, bluring))
    # Creation of the mask
    mask = cv2.inRange(image, lo, hi)
    # Erode and dilate to further remove imperfections
    mask = cv2.erode(mask, None, iterations=morphiterations)
    mask = cv2.dilate(mask, None, iterations=morphiterations)
    # Return mask
    return mask


def findElementFromMask(mask, minRadius=30):
    """
    Finds the center and radius of the minimum enclosing circle of the largest contour in the given mask.

    Args:
        mask (numpy.ndarray): The binary mask image.
        minRadius (int, optional): The minimum radius of the enclosing circle. Defaults to 30.

    Returns:
        tuple: A tuple containing the center coordinates (x, y) and the radius of the minimum enclosing circle.
               If the radius is less than or equal to minRadius, returns None.
    """
    # We search all the contours in the mask
    elements, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # If we have found some elements
    if len(elements) > 0:
        # Find the largest contour
        c = max(elements, key=cv2.contourArea)
        # Get the center and the radius of the minimum enclosing circle
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        # Return the center and radius if the radius is greater than 30
        return ((x, y), radius) if radius >= minRadius else (None, None)
    return (None, None)


def drawCircle(image, center, radius, color=(0, 0, 255), thickness=2):
    """
    Draw a circle on an image.

    Args:
        image (numpy.ndarray): The image on which to draw the circle.
        center (tuple): The center of the circle.
        radius (int): The radius of the circle.
        color (tuple, optional): The color of the circle. Defaults to red.
        thickness (int, optional): The thickness of the circle. Defaults to 2.
    """
    x, y = center
    draw = cv2.circle(image, (int(x), int(y)), int(radius), color, thickness)
    return draw


def drawPointAndText(image, point, text, color=(0, 0, 255)) :
    """
    Draws a point and text on the given image.

    Parameters:
        image: The image on which to draw the point and text.
        point: The coordinates of the point (x, y).
        text: The text to be displayed.
        color: The color of the point and text. Defaults to red.
    """
    x, y = point
    draw = cv2.circle(image, (int(x), int(y)), 5, color, 10)
    draw = cv2.line(draw, (int(x), int(y)), (int(x)+150, int(y)), color, 2)
    draw = cv2.putText(draw, text, (int(x)+10, int(y) -10), cv2.FONT_HERSHEY_DUPLEX, 0.8, color, 1, cv2.LINE_AA)
    return draw