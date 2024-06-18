import sys
import cv2 as cv
import numpy as np
import nptyping
import math

import mmap
import posix_ipc
import os
from time import sleep
from enum import Enum, IntEnum
import struct
import random

import signal

import argparse
import logging

import time

from nptyping import NDArray
from typing import *

################################
####### GLOBAL VARIABLES #######
################################

class State(IntEnum):
    Disarmed = 0
    Grounded = 1
    TakingOff = 2
    Flying = 3
    Landing = 4

state = State.Disarmed

class FlyingState(IntEnum):
    Hovering = 0
    FollowingObject = 1
    FlyingForward = 2
    StabilizedHover = 3

flyingState = FlyingState.Hovering

signals_to_handle = [
    signal.SIGINT,
    signal.SIGTERM,
    signal.SIGHUP,
    signal.SIGUSR1,
    signal.SIGUSR2,
    signal.SIGQUIT,
    signal.SIGABRT
]





#################################################################
##################  PHYSICS CONSTANTS - START  ##################
#################################################################
PI = 3.14159
G = 9.81 # m/s^2
RHO = 1.225 # kg/m^3
DEG_TO_RAD = 180/PI
#################################################################
###################  PHYSICS CONSTANTS - END  ###################
#################################################################





#################################################################
###############  USER EDITABLE VARIABLES - START  ###############
#################################################################
TAKEOFF_TIME = 0.6 # The time it takes for the drone to take off
TAKEOFF_HEIGHT = 0.1 # The height the drone will take off to in meters
MASS = 0.31 # The mass of the drone in kg
LANDING_TIME = 3 # The drone will be descending for this time

MAX_FORWARD_ACCELERATION = 2 # m/s^2
MAX_SIDEWAYS_ACCELERATION = 2 # m/s^2
MAX_VERTICAL_ACCELERATION = 2 # m/s^2
MAX_ANGULAR_ACCELERATION = 90 # degrees/s^2

VERTICAL_ACCELERATION_OFFSET = 0.5 # The drone hovers at this acceleration
#################################################################
################  USER EDITABLE VARIABLES - END  ################
#################################################################





TAKEOFF_TIME1 = 2 * TAKEOFF_TIME / 3
TAKEOFF_TIME2 = TAKEOFF_TIME / 3
MASS_N = MASS * G # The weight of the drone in Newtons
LANDING_TIME1 = LANDING_TIME / 3


PURPLE = ((130, 150, 150), (140, 255, 255))
ORANGE = ((10, 150, 150), (20, 255, 255))
RED = ((170, 125, 125), (10, 255, 255))
GREEN = ((55, 150, 150), (65, 255, 255))
BLUE = ((100, 110, 110), (115, 255, 255))

running = True # The main loop of the program





#################################################################
####################  SHARED MEMORY - START  ####################
#################################################################
memory = None
map_file = None
values = None
#################################################################
#####################  SHARED MEMORY - END  #####################
#################################################################
cap = None

forward = 0 # Move front or back
sideways = 0 # Move left or right
vertical = -10 # Move up or down
angle = 0 # Turn left or right

height = 0 # The calculated height of the drone

image = None # The current image from the camera

dt = 0 # The time difference between to Update calls





#################################################################
############  TAKEOFF AND LANDING VARIABLES - START  ############
#################################################################
takeoffCnt = 0
takeoffAccel1: float = 0
takeoffAccel2: float = 0

landingCnt = 0
landingAccel1: float = 0
landingAccel2: float = 0
#################################################################
#############  TAKEOFF AND LANDING VARIABLES - END  #############
#################################################################





#################################################################
#################  UTILITY FUNCTIONS - START  ###################
#################################################################
"""
The code below is licensed under the MIT License.


Copyright MIT and Harvey Mudd College
MIT License
Summer 2020

Contains helper functions to support common operations.
"""
class ColorBGR(Enum):
    """
    Common colors defined in the blue-green-red (BGR) format, with each channel
    ranging from 0 to 255 inclusive.
    """

    blue = (255, 0, 0)
    light_blue = (255, 255, 0)
    green = (0, 255, 0)
    dark_green = (0, 127, 0)
    yellow = (0, 255, 255)
    orange = (0, 127, 255)
    red = (0, 0, 255)
    pink = (255, 0, 255)
    purple = (255, 0, 127)
    black = (0, 0, 0)
    dark_gray = (63, 63, 63)
    gray = (127, 127, 127)
    light_gray = (191, 191, 191)
    white = (255, 255, 255)
    brown = (0, 63, 127)

def clamp(value: float, min: float, max: float) -> float:
    """
    Clamps a value between a minimum and maximum value.

    Args:
        value: The input to clamp.
        min: The minimum allowed value.
        max: The maximum allowed value.

    Returns:
        The value saturated between min and max.

    Example::

        # a will be set to 3
        a = rc_utils.clamp(3, 0, 10)

        # b will be set to 0
        b = rc_utils.remap_range(-2, 0, 10)

        # c will be set to 10
        c = rc_utils.remap_range(11, 0, 10)
    """
    return min if value < min else max if value > max else value

def remap_range(
    val: float,
    old_min: float,
    old_max: float,
    new_min: float,
    new_max: float,
    saturate: bool = False,
) -> float:
    """
    Remaps a value from one range to another range.

    Args:
        val: A number form the old range to be rescaled.
        old_min: The inclusive 'lower' bound of the old range.
        old_max: The inclusive 'upper' bound of the old range.
        new_min: The inclusive 'lower' bound of the new range.
        new_max: The inclusive 'upper' bound of the new range.
        saturate: If true, the new_min and new_max limits are enforced.

    Note:
        min need not be less than max; flipping the direction will cause the sign of
        the mapping to flip.  val does not have to be between old_min and old_max.

    Example::

        # a will be set to 25
        a = rc_utils.remap_range(5, 0, 10, 0, 50)

        # b will be set to 975
        b = rc_utils.remap_range(5, 0, 20, 1000, 900)

        # c will be set to 30
        c = rc_utils.remap_range(2, 0, 1, -10, 10)

        # d will be set to 10
        d = rc_utils.remap_range(2, 0, 1, -10, 10, True)
    """
    old_span: float = old_max - old_min
    new_span: float = new_max - new_min
    new_val: float = new_min + new_span * (float(val - old_min) / float(old_span))

    # If saturate is true, enforce the new_min and new_max limits
    if saturate:
        if new_min < new_max:
            return clamp(new_val, new_min, new_max)
        else:
            return clamp(new_val, new_max, new_min)

    return new_val

def find_contours(
    color_image: Any,
    hsv_lower: Tuple[int, int, int],
    hsv_upper: Tuple[int, int, int],
) -> List[NDArray]:
    """
    Finds all contours of the specified color range in the provided image.

    Args:
        color_image: The color image in which to find contours,
            with pixels represented in the bgr (blue-green-red) format.
        hsv_lower: The lower bound for the hue, saturation, and value of colors
            to contour.
        hsv_upper: The upper bound for the hue, saturation, and value of the colors
            to contour.

    Returns:
        A list of contours around the specified color ranges found in color_image.

    Note:
        Each channel in hsv_lower and hsv_upper ranges from 0 to 255.

    Example::

        # Define the lower and upper hsv ranges for the color blue
        BLUE_HSV_MIN = (90, 50, 50)
        BLUE_HSV_MAX = (110, 255, 255)

        # Extract contours around all blue portions of the current image
        contours = rc_utils.find_contours(
            rc.camera.get_color_image(), BLUE_HSV_MIN, BLUE_HSV_MAX
        )
    """
    assert (
        0 <= hsv_lower[0] <= 179 and 0 <= hsv_upper[0] <= 179
    ), f"The hue of hsv_lower ({hsv_lower}) and hsv_upper ({hsv_upper}) must be in the range 0 to 179 inclusive."

    assert (
        0 <= hsv_lower[1] <= 255 and 0 <= hsv_upper[1] <= 255
    ), f"The saturation of hsv_lower ({hsv_lower}) and hsv_upper ({hsv_upper}) must be in the range 0 to 255 inclusive."

    assert (
        0 <= hsv_lower[0] <= 255 and 0 <= hsv_upper[0] <= 255
    ), f"The value of hsv_lower ({hsv_lower}) and hsv_upper ({hsv_upper}) must be in the range 0 to 255 inclusive."

    assert (
        hsv_lower[1] <= hsv_upper[1]
    ), f"The saturation channel of hsv_lower ({hsv_lower}) must be less than that of hsv_upper ({hsv_upper})."

    assert (
        hsv_lower[2] <= hsv_upper[2]
    ), f"The value channel of hsv_lower ({hsv_lower}) must be less than that of of hsv_upper ({hsv_upper})."

    # Convert the image from a blue-green-red pixel representation to a
    # hue-saturation-value representation
    hsv_image = cv.cvtColor(color_image, cv.COLOR_BGR2HSV)

    # Create a mask containing the pixels in the image with hsv values between
    # hsv_lower and hsv_upper.
    mask: NDArray
    if hsv_lower[0] <= hsv_upper[0]:
        mask = cv.inRange(hsv_image, hsv_lower, hsv_upper)

    # If the color range passes the 255-0 boundary, we must create two masks
    # and merge them
    else:
        mask1 = cv.inRange(hsv_image, hsv_lower, (255, hsv_upper[1], hsv_upper[2]))
        mask2 = cv.inRange(hsv_image, (0, hsv_lower[1], hsv_lower[2]), hsv_upper)
        mask = cv.bitwise_or(mask1, mask2)

    # Find and return a list of all contours of this mask
    return cv.findContours(mask, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)[0]

def get_largest_contour(
    contours: List[NDArray], min_area: int = 30
) -> Optional[NDArray]:
    """
    Finds the largest contour with size greater than min_area.

    Args:
        contours: A list of contours found in an image.
        min_area: The smallest contour to consider (in number of pixels)

    Returns:
        The largest contour from the list, or None if no contour was larger
        than min_area.

    Example::

        # Extract the blue contours
        BLUE_HSV_MIN = (90, 50, 50)
        BLUE_HSV_MAX = (110, 255, 255)
        contours = rc_utils.find_contours(
            rc.camera.get_color_image(), BLUE_HSV_MIN, BLUE_HSV_MAX
        )

        # Find the largest contour
        largest_contour = rc_utils.get_largest_contour(contours)
    """
    # Check that the list contains at least one contour
    if len(contours) == 0:
        return None

    # Find and return the largest contour if it is larger than min_area
    greatest_contour = max(contours, key=cv.contourArea)
    if cv.contourArea(greatest_contour) < min_area:
        return None

    return greatest_contour

def draw_contour(
    color_image: Any,
    contour: NDArray,
    color: Tuple[int, int, int] = ColorBGR.green.value,
) -> None:
    """
    Draws a contour on the provided image.

    Args:
        color_image: The color image on which to draw the contour.
        contour: The contour to draw on the image.
        color: The color to draw the contour, specified as
            blue-green-red channels each ranging from 0 to 255 inclusive.

    Example::

        image = rc.camera.get_color_image()

        # Extract the largest blue contour
        BLUE_HSV_MIN = (90, 50, 50)
        BLUE_HSV_MAX = (110, 255, 255)
        contours = rc_utils.find_contours(image, BLUE_HSV_MIN, BLUE_HSV_MAX)
        largest_contour = rc_utils.get_largest_contour(contours)

        # Draw this contour onto image
        if (largest_contour is not None):
            draw_contour(image, largest_contour)
    """
    for channel in color:
        assert (
            0 <= channel <= 255
        ), f"Each channel in color ({color}) must be in the range 0 to 255 inclusive."

    cv.drawContours(color_image, [contour], 0, color, 3)

def draw_circle(
    color_image: Any,
    center: Tuple[int, int],
    color: Tuple[int, int, int] = ColorBGR.yellow.value,
    radius: int = 6,
) -> None:
    """
    Draws a circle on the provided image.

    Args:
        color_image: The color image on which to draw the contour.
        center: The pixel (row, column) of the center of the image.
        color: The color to draw the circle, specified as
            blue-green-red channels each ranging from 0 to 255 inclusive.
        radius: The radius of the circle in pixels.

    Example::

        image = rc.camera.get_color_image()

        # Extract the largest blue contour
        BLUE_HSV_MIN = (90, 50, 50)
        BLUE_HSV_MAX = (110, 255, 255)
        contours = rc_utils.find_contours(image, BLUE_HSV_MIN, BLUE_HSV_MAX)
        largest_contour = rc_utils.get_largest_contour(contours)

        # Draw a dot at the center of this contour in red
        if (largest_contour is not None):
            center = get_contour_center(contour)
            draw_circle(image, center, rc_utils.ColorBGR.red.value)
    """
    for channel in color:
        assert (
            0 <= channel <= 255
        ), f"Each channel in color ({color}) must be in the range 0 to 255 inclusive."

    assert (
        0 <= center[0] < color_image.shape[0]
    ), f"center[0] ({center[0]}) must be a pixel row index in color_image."
    assert (
        0 <= center[1] < color_image.shape[1]
    ), f"center[1] ({center[1]}) must be a pixel column index in color_image."
    assert radius > 0, f"radius ({radius}) must be a positive integer."

    # cv.circle expects the center in (column, row) format
    cv.circle(color_image, (center[1], center[0]), radius, color, -1)

def get_contour_center(contour: NDArray) -> Optional[Tuple[int, int]]:
    """
    Finds the center of a contour from an image.

    Args:
        contour: The contour of which to find the center.

    Returns:
        The (row, column) of the pixel at the center of the contour, or None if the
        contour is empty.

    Example::

        # Extract the largest blue contour
        BLUE_HSV_MIN = (90, 50, 50)
        BLUE_HSV_MAX = (110, 255, 255)
        contours = rc_utils.find_contours(
            rc.camera.get_color_image(), BLUE_HSV_MIN, BLUE_HSV_MAX
        )
        largest_contour = rc_utils.get_largest_contour(contours)

        # Find the center of this contour if it exists
        if (largest_contour is not None):
            center = rc_utils.get_contour_center(largest_contour)
    """
    M = cv.moments(contour)

    # Check that the contour is not empty
    # (M["m00"] is the number of pixels in the contour)
    if M["m00"] <= 0:
        return None

    # Compute and return the center of mass of the contour
    center_row = round(M["m01"] / M["m00"])
    center_column = round(M["m10"] / M["m00"])
    return (center_row, center_column)

def get_contour_area(contour: NDArray) -> float:
    """
    Finds the area of a contour from an image.

    Args:
        contour: The contour of which to measure the area.

    Returns:
        The number of pixels contained within the contour

    Example::

        # Extract the largest blue contour
        BLUE_HSV_MIN = (90, 50, 50)
        BLUE_HSV_MAX = (110, 255, 255)
        contours = rc_utils.find_contours(
            rc.camera.get_color_image(), BLUE_HSV_MIN, BLUE_HSV_MAX
        )
        largest_contour = rc_utils.get_largest_contour(contours)

        # Find the area of this contour (will evaluate to 0 if no contour was found)
        area = rc_utils.get_contour_area(contour)
    """
    return cv.contourArea(contour)
"""
The code above is licensed under the MIT License.
"""
def findContour(image, *colors):
    contours = []
    for color in colors:
        contour = get_largest_contour(find_contours(image, color[0], color[1]))
        if contour is not None:
            contours.append(contour)
            draw_contour(image, contour, color[1])
            center = get_contour_center(contour)
            draw_circle(image, center)
    if len(contours) > 0:
        largest_contour = max(contours, key=get_contour_area)
        largest_center = get_contour_center(largest_contour)
        return largest_contour, largest_center
    else:
        return None, None
#################################################################
##################  UTILITY FUNCTIONS - END  ####################
#################################################################





#################################################################
#############  LESS INTERESTING FUNCTIONS - START  ##############
#################################################################
def cleanup():
    global running
    global cap
    global map_file
    global memory

    running = False
    print("Program stopped running")
    logging.debug("\tDisarming...")

    Disarm()

    logging.debug("\tDrone disarmed")
    logging.debug("\tReleasing camera...")
    cap.release()
    cv.destroyAllWindows()

    logging.debug("\tCamera released")
    logging.debug("\tClosing shared memory...")

    map_file.close()
    
    logging.debug("\tShared memory closed")
    print("Exiting...")
    sys.exit(0)

def handle_signal(signum, frame):
    """Handles many signals that stop program execution by cleaning
    up and exiting the program.\nVERY IMPORTANT! Under no circumstances
    should the program exit without calling cleanup or disarming the
    drone!"""
    print(f"\nDetected signal {signum}!\nCleaning up...")
    cleanup()

def passValues(*inputs):
    """Passes the given values to the shared memory, effectively transmitting them to the drone."""
    global values
    if len(inputs) != 16:
        logging.debug("\tError: You must pass 16 values!")
        cleanup()
        return
    values = list(inputs) + [0] * (16 - len(inputs))
    map_file.seek(0)  # Go back to the beginning of the mmap
    map_file.write(struct.pack('i'*16, *values))
#################################################################
##############  LESS INTERESTING FUNCTIONS - END  ###############
#################################################################





#################################################################
##################  BASIC FUNCTIONS - START  ####################
#################################################################
def Arm():
    """Arms the drone by lowering the throttle to the minimum value
    and setting AUX1 to high."""
    passValues(0, 0, 0, -32768, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
    sleep(0.5)
    passValues(0, 0, 0, -32768, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0)

def Disarm():
    """Disarms the drone by lowering the throttle to the minimum value
    and setting AUX1 to low."""
    passValues(0, 0, 0, -32768, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)

def ZeroThrottle():
    """Sets the throttle to the minimum value, with the drone still armed."""
    global forward
    global angle
    global vertical

    forward = 0
    angle = 0
    vertical = -10
#################################################################
###################  BASIC FUNCTIONS - END  #####################
#################################################################





#################################################################
###############  CONVERSION FUNCTIONS - START  ##################
#################################################################
const = (PI * RHO * (0.0762**2) * 0.0635) / (3600 * 4 * 50) # * (RPM**2)
def Thrust(rpm: int):
    """Calculates the thrust produced by a motor spinning at a given RPM."""
    # Simplified thrust formula: T = (pi * rho * D^2 * n^2 * P) / 4
    global const
    return const * (rpm ** 2)

const2 = math.sqrt((4 * 50 * 3600) / (PI * RHO * (0.0762 ** 2) * 0.0635)) # * sqrt(RPM)
def ThrustToRPM(thrust: float):
    """Calculates the RPM of a motor needed to produce the given thrust by it."""
    global const2
    return int((math.sqrt(thrust)) * const2)

const3 = 1/(10196720000**1.011358)
def RPMtoThrottleCRSF(rpm: int):
    """Converts the RPM of a motor to a CRSF throttle value [1000, 2000]."""
    global const3
    return int(270782500 + (-270781529.4186)/(1 + rpm**1.011358 * const3))

def CRSFtoInt(value: int):
    """Converts from CRSF [1000, 2000] to int16 [-32768, 32767]."""
    return int(52.88747*value - 79333.9)
               
def intToCRSF(value: int):
    """Converts from int16 [-32768, 32767] to CRSF [1000, 2000]."""
    return int(0.01890788*value + 1500.051)

def intToDegPerSec(value: int):
    """Converts Pitch, Roll, and Yaw values [-32768, 32767] to degrees per second."""
    return 0.02534762*value + 0.02909812

const4 = 0.02909812/0.02534762
def degPerSecToInt(value: float):
    """Converts degrees per second to int16 [-32768, 32767]."""
    return int(value/0.02534762 - const4)
#################################################################
################  CONVERSION FUNCTIONS - END  ###################
#################################################################





#################################################################
############  TAKEOFF AND LANDING FUNCTIONS - START  ############
#################################################################
def CalculateTakeoff(h: float, t: float):
    """Calculates the thrust needed for the drone to take off to a given
    height `h` in a given time `t`.""" 
    global height
    global takeoffAccel1
    global takeoffAccel2

    a1 = (3 * h)/(t ** 2)

    # This is not necessary for takeoff, but it is useful for debugging
    takeoffThrust1 = MASS * (a1 + G)
    takeoffRPM1 = ThrustToRPM(takeoffThrust1/4) # Thrust per motor
    takeoffThrottle1 = CRSFtoInt(RPMtoThrottleCRSF(takeoffRPM1))
    u1 = (2 * a1 * t)/3

    a2  = -3 * u1/t # This, however, is necessary for takeoff
    takeoffThrust2 = MASS * (a2 + G)
    takeoffRPM2 = ThrustToRPM(takeoffThrust2/4) # Thrust per motor
    takeoffThrottle2 = CRSFtoInt(RPMtoThrottleCRSF(takeoffRPM2))
    # End of unnecessary calculations

    takeoffAccel1 = clamp((a1/MAX_VERTICAL_ACCELERATION), -1, 1)
    takeoffAccel2 = clamp((a2/MAX_VERTICAL_ACCELERATION), -1, 1)

    logging.debug(f"\tTakeoff acceleration: {a1}")
    logging.debug(f"\tClamped takeoff acceleration: {takeoffAccel1}")
    logging.debug(f"\tTakeoff speed: {u1}", )
    logging.debug(f"\tTakeoff Thrust: {takeoffThrust1}")
    logging.debug(f"\tTakeoff RPM: {takeoffRPM1}")
    logging.debug(f"\tTakeoff Throttle CRSF: {intToCRSF(takeoffThrottle1)}")
    logging.debug(f"\tTakeoff Throttle: {takeoffThrottle1}")
    logging.debug(f"\tTarget thrust at takeoff: {takeoffThrust1}")
    logging.debug(f"\tPredicted thrust at takeoff: {Thrust(takeoffRPM1) * 4}")
    
    logging.debug("\n")

    logging.debug(f"\tTakeoff acceleration: {a2}")
    logging.debug(f"\tClamped takeoff acceleration: {takeoffAccel2}")
    logging.debug(f"\tTakeoff Thrust: {takeoffThrust2}")
    logging.debug(f"\tTakeoff RPM: {takeoffRPM2}")
    logging.debug(f"\tTakeoff Throttle CRSF: {intToCRSF(takeoffThrottle2)}")
    logging.debug(f"\tTakeoff Throttle: {takeoffThrottle2}")
    logging.debug(f"\tTarget thrust at takeoff: {takeoffThrust2}")
    logging.debug(f"\tPredicted thrust at takeoff: {Thrust(takeoffRPM2) * 4}")

    logging.debug("\n")

def Takeoff(takeoffStage: int):
    global forward
    global vertical
    global angle

    global takeoffAccel1
    global takeoffAccel2

    if takeoffStage == 1:
        logging.debug(f"\tSetting acceleration to {takeoffAccel1} for takeoff stage 1...")
        forward = 0
        vertical = takeoffAccel1
        angle = 0
    elif takeoffStage == 2:
        logging.debug(f"\tSetting acceleration to {takeoffAccel2} for takeoff stage 2...")
        forward = 0
        vertical = takeoffAccel2
        angle = 0
    else:
        logging.debug(f"\tInvalid takeoff stage {takeoffStage}. Must be 1 or 2")

# TODO: Actually calculate landing thrust
def CalculateLanding():
    """Calculates the thrust needed for the drone to land relatively slowly.""" 
    global landingAccel1
    global landingAccel2

    # landingThrottle1 = CRSFtoInt(RPMtoThrottle(ThrustToRPM(MASS_N/4)))
    # landingThrottle2 = CRSFtoInt(RPMtoThrottle(ThrustToRPM(MASS_N/4)))

    landingAccel1 = -0.2
    landingAccel2 = -0.1

    logging.debug(f"Landing Acceleration 1: {landingAccel1}")
    logging.debug(f"Landing Acceleration 2: {landingAccel2}")
    logging.debug("\n")

def Land(landingStage: int):
    global forward
    global vertical
    global angle
    
    global landingAccel1
    global landingAccel2
    if landingStage == 1:
        logging.debug(f"\tSetting acceleration to {landingAccel1} for landing stage 1...")
        forward = 0
        vertical = landingAccel1
        angle = 0
    elif landingStage == 2:
        logging.debug(f"\tSetting acceleration to {landingAccel2} for landing stage 2...")
        forward = 0
        vertical = landingAccel2
        angle = 0
    else:
        logging.debug(f"\tInvalid landing stage {landingStage}. Must be 1 or 2")
#################################################################
#############  TAKEOFF AND LANDING FUNCTIONS - END  #############
#################################################################





#################################################################
##################  CONTROL FUNCTIONS - START  ##################
#################################################################
def control():
    """Convert the forward, angle, and vertical values to pitch,
    yaw, roll and throttle values and pass them to the drone."""
    global forward
    global sideways
    global vertical
    global angle

    if forward > 1 or forward < -1:
        print(f"\tInvalid forward value {forward}. Must be between -1 and 1.")
        cleanup()
    elif angle > 1 or angle < -1:
        print(f"\tInvalid angle value {angle}. Must be between -1 and 1.")
        cleanup()
    elif (vertical > 1 or vertical < -1) and not vertical == -10:
        print(f"\tInvalid vertical value {vertical}. Must be between -1 and 1.")
        cleanup()
    elif sideways > 1 or sideways < -1:
        print(f"\tInvalid sideways value {sideways}. Must be between -1 and 1.")
        cleanup()

    a_x = forward*MAX_FORWARD_ACCELERATION
    a_y = sideways*MAX_SIDEWAYS_ACCELERATION
    a_z = (vertical+VERTICAL_ACCELERATION_OFFSET)*MAX_VERTICAL_ACCELERATION
    w_y = angle*MAX_ANGULAR_ACCELERATION

    logging.debug(f"\ta_x: , {a_x}, a_y: {a_y}, a_z: , {a_z}, w_y: , {w_y}")

    _thrustConstXZ = 0
    ThrustXZ = 0
    _thrustConstYZ = 0
    ThrustYZ = 0
    Theta = 0
    Phi = 0

    if vertical == -10:
        Thrust = 0
        Theta = 0
        rpm = 0
        yaw = 0
        pitch = 0
        roll = 0
        throttle = -32760
    else:
        _thrustConstXZ = math.sqrt((a_z+G)**2 + a_x**2)
        ThrustXZ = MASS * _thrustConstXZ
        Theta = math.asin(a_x/_thrustConstXZ) * DEG_TO_RAD

        logging.debug(f"\tTheta: {Theta}")

        _thrustConstYZ = math.sqrt((a_z+G)**2 + a_y**2)
        ThrustYZ = MASS * _thrustConstYZ
        Phi = math.asin(a_y/_thrustConstYZ) * DEG_TO_RAD

        logging.debug(f"\tPhi: {Phi}")

        Thrust = MASS * math.sqrt(a_x**2 + a_y**2 + (a_z+G)**2)

        rpm = ThrustToRPM(Thrust/4) # Thrust per motor
        throttle = CRSFtoInt(RPMtoThrottleCRSF(rpm))

        yaw = degPerSecToInt(w_y)
        pitch = degPerSecToInt(Theta)
        roll = degPerSecToInt(Phi)
        # We are flying in ANGLE mode, which means that the stick
        # position is the pitch of the drone, not the acceleration
        # of the pitch. Thus, degrees per second is not actually
        # degrees per second, but rather just degrees.
        # The same applies to roll


    logging.debug(f"\tThrust: {Thrust}")
    logging.debug(f"\tThrust XZ: {ThrustXZ}")
    logging.debug(f"\tThrust YZ: {ThrustYZ}")
    logging.debug(f"\tRPM: {rpm}")
    logging.debug(f"\tThrottle: {throttle}")
    logging.debug(f"\tThrottle CRSF: {intToCRSF(throttle)}")
    logging.debug(f"\tPitch angle: {Theta}")
    logging.debug(f"\tRoll angle: {Phi}")
    logging.debug(f"\tPitch: {pitch}")
    logging.debug(f"\tYaw: {yaw}")
    logging.debug(f"\tRoll: {roll}")

    passValues(yaw, pitch, roll, throttle, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0)

def Hover():
    global forward
    global sideways
    global vertical
    global angle

    forward = 0
    sideways = 0
    vertical = 0
    angle = 0

def flyForward():
    global forward
    global sideways
    global vertical
    global angle

    forward = 1
    sideways = 0
    vertical = 0
    angle = 0

def followHoops():
    global image
    global angle
    global height
    
    contour, center = findContour(image, RED, GREEN, BLUE)
    if center is None:
        return
    else:
        dx = remap_range(center[1], 0, 640, -32768, 32767) # Center is (y, x)
        dy = remap_range(center[0], 0, 480, 32767, -32768) # dy is positive when the hoop is above the center
        print("dx: ", dx, "dy: ", dy)

    # Display the image
    cv.imshow("frame", image)
    print("Center: ", center)

def followTarget(*colors):
    global image
    global forward
    global angle
    global vertical

    contour, center = findContour(image, *colors)

    if center is None:
        return
    else:
        dx = remap_range(center[1], 0, 640, -1, 1, True)
        dy = remap_range(center[0], 0, 480, 1, -1, True)

        angle = dx
        vertical = dy
        forward = 0.25

        logging.debug("\n\n")
        logging.debug(f"\tAngle: {angle}")
        logging.debug(f"\tVertical: {vertical}\n\n")

def Stabilize():
    global forward
    global sideways
    global vertical
    global angle
    global image

    forward = 0
    sideways = 0
    vertical = 0
    angle = 0

    contour, center = findContour(image, GREEN)
    if center is None:
        return
    else:
        dx = remap_range(center[1], 0, 640, -1, 1, True)
        dy = remap_range(center[0], 0, 480, 1, -1, True)

        forward = 0
        sideways = dx
        vertical = dy
        angle = 0

        logging.debug("\n\n")
        logging.debug(f"\tSideways: {sideways}")
        logging.debug(f"\tVertical: {vertical}\n\n")
        logging.debug("\n\n")
#################################################################
###################  CONTROL FUNCTIONS - END  ###################
#################################################################






def Awake():
    """Initialization function that is called first even before Start()"""
    print("Program started")
    logging.debug("\tInitializing signal handlers...")

    for sig in signals_to_handle:
        signal.signal(sig, handle_signal)

    logging.debug("\tSignal handlers initialized")

def Start():
    """Initialization function that is called once when the program starts."""
    global cap
    global map_file
    global values
    global memory
    global state
    
    logging.debug("\tInitializing shared memory...")
    memory = posix_ipc.SharedMemory("/myshm")
    map_file = mmap.mmap(memory.fd, memory.size)
    values = struct.unpack('i'*16, map_file.read(64))
    
    logging.debug("\tShared memory initialized")
    logging.debug("\tInitializing camera...")
    
    cap = cv.VideoCapture("/dev/video2")
    
    logging.debug("\tCamera initialized")
    logging.debug("\tDisarming drone...")

    Disarm()
    state = State.Disarmed

    logging.debug("\tDrone disarmed")
    logging.debug("\tGetting first frame...")

    ret, _ = cap.read()
    if ret:
        logging.debug("\tFirst frame captured")
        logging.debug("\tArming drone...")
        Arm()  # Arm the drone
        state = State.Grounded
        logging.debug("\tDrone armed")
        CalculateTakeoff(TAKEOFF_HEIGHT, TAKEOFF_TIME)
        CalculateLanding()
        print("\nDrone initialized!\n\n\n")
    else:
        print("Failed get first frame!\nThere is a problem with the camera", file=sys.stderr)
        print("Cleaning up...")
        cleanup()
        return

def UpdateHelp():
    global running
    global dt

    initialTime = time.perf_counter()
    while running:
        # This function calls update
        Update()

        dt = time.perf_counter() - initialTime
        initialTime = time.perf_counter()
    
    cleanup()

def Update():
    global running
    global cap
    global image
    global values
    global state
    global flyingState
    global dt

    global takeoffCnt

    global landingCnt

    ret, image = cap.read()
    if not ret:
        print("Error: failed to capture image")
        running = False
        return
    else:
        # logging.debug(f"dt: {dt}")

        keyPressed = cv.waitKey(1)


        if state == State.Disarmed:
            if keyPressed == ord('r'):
                try:
                    Arm()
                except Exception as e:
                    print(f"An error occurred while calling Arm() when Disarmed: {e}")
                    cleanup()
                state = State.Grounded
                print("Armed")
                
        elif state == State.Grounded:
            if keyPressed == ord('w'):
                state = State.TakingOff
                takeoffCnt = 0
                print("Taking off...")
            elif keyPressed == ord('r'):
                try:
                    Disarm()
                except Exception as e:
                    print(f"An error occurred while calling Disarm() with the drone Grounded: {e}")
                    cleanup()
                # No point in using try-except here
                # if Disarm() fails, there is a serious problem
                # cleanup() will just call Disarm() again
                # but for consistency, we will use try-except
                state = State.Disarmed
                print("Disarmed")
            else:
                try:
                    ZeroThrottle()
                except Exception as e:
                    print(f"An error occurred while calling ZeroThrottle() with the drone Grounded: {e}")
                    cleanup()

        elif state == State.TakingOff:
            if takeoffCnt < TAKEOFF_TIME1:
                try:
                    Takeoff(1)
                except Exception as e:
                    print(f"An error occurred while calling Takeoff(1): {e}")
                    cleanup()
            elif takeoffCnt < TAKEOFF_TIME:
                try:
                    Takeoff(2)
                except Exception as e:
                    print(f"An error occurred while calling Takeoff(2): {e}")
                    cleanup()
            else:
                state = State.Flying
                flyingState = FlyingState.Hovering
                takeoffCnt = 0
                print("Flying...")
                Hover()

            takeoffCnt += dt

        elif state == State.Flying:
            if keyPressed == ord('s'):
                state = State.Landing
                landingCnt = 0
                print("Landing...")
            elif keyPressed == ord('f'):
                flyingState = FlyingState.FollowingObject
                print("Following object")
            elif keyPressed == ord('h'):
                flyingState = FlyingState.Hovering
                print("Hovering")
            elif keyPressed == ord('w'):
                flyingState = FlyingState.FlyingForward
                print("Flying forward")
            elif keyPressed == ord('a'):
                flyingState = FlyingState.StabilizedHover
                print("Stabilized hover")
            

            if flyingState == FlyingState.Hovering:
                logging.debug("Hovering")
                try:
                    Hover()
                except Exception as e:
                    print(f"An error occurred: {e}")
                    cleanup()
            elif flyingState == FlyingState.FollowingObject:
                try:
                    followTarget(BLUE)
                    pass
                except Exception as e:
                    print(f"An error occurred: {e}")
                    cleanup()
            elif flyingState == FlyingState.FlyingForward:
                try:
                    flyForward()
                except Exception as e:
                    print(f"An error occurred: {e}")
                    cleanup()
            elif flyingState == FlyingState.StabilizedHover:
                try:
                    Stabilize()
                except Exception as e:
                    print(f"An error occurred: {e}")
                    cleanup()

        elif state == State.Landing:
            if landingCnt < LANDING_TIME1:
                try:
                    Land(1)
                except Exception as e:
                    print(f"An error occurred while calling Land(1): {e}")
                    cleanup()
            elif landingCnt < LANDING_TIME:
                try:
                    Land(2)
                except Exception as e:
                    print(f"An error occurred while calling Land(2): {e}")
                    cleanup()
            else:
                ZeroThrottle()
                state = State.Grounded
                landingCnt = 0
                print("Landed")
            
            landingCnt += dt


        try:
            control()
        except Exception as e:
            print(f"An error occurred while calling control(): {e}")
            cleanup()


        if keyPressed == ord('q'):
            running = False
        
        cv.imshow("frame", image)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="A program that controls a drone and makes it semi-autonomous."
    )
    parser.add_argument("-v", "--verbose", help="increase output verbosity", action="store_true")
    args = parser.parse_args()
    if args.verbose:
        logging.basicConfig(level=logging.DEBUG)

    logging.debug("\tVerbose mode")
    Awake()
    Start()
    UpdateHelp()
