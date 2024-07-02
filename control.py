import logging
from basic import cleanup, passValues
from config import *
from config import Config as c
from conversions import *
from utils import *
import cv2 as cv

def control():
    """Convert the forward, angle, and vertical values to c.pitch,
    yaw, roll and throttle values and pass them to the drone."""

    if c.state == State.Disarmed:
        passValues(0, 0, 0, -32760, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
        logging.debug("\tControl() called while disarmed")
        return

    if c.forward > 1 or c.forward < -1:
        print(f"\tInvalid forward value {c.forward}. Must be between -1 and 1.")
        cleanup()
        return
    elif c.angle > 1 or c.angle < -1:
        print(f"\tInvalid angle value {c.angle}. Must be between -1 and 1.")
        cleanup()
        return
    elif (c.vertical > 1 or c.vertical < -1) and not c.vertical == -10:
        print(f"\tInvalid vertical value {c.vertical}. Must be between -1 and 1.")
        cleanup()
        return
    elif c.sideways > 1 or c.sideways < -1:
        print(f"\tInvalid sideways value {c.sideways}. Must be between -1 and 1.")
        cleanup()
        return

    c.a_x = c.forward*MAX_FORWARD_ACCELERATION
    c.a_y = c.sideways*MAX_SIDEWAYS_ACCELERATION
    c.a_z = (c.vertical+VERTICAL_ACCELERATION_OFFSET)*MAX_VERTICAL_ACCELERATION
    c.w_y = c.angle*MAX_ANGULAR_ACCELERATION

    _thrustConstXZ = 0
    c.ThrustXZ = 0
    _thrustConstYZ = 0
    c.ThrustYZ = 0
    c.Theta = 0
    c.Phi = 0

    if c.vertical == -10:
        c.Thrust = 0
        c.Theta = 0
        c.rpm = 0
        c.yaw = 0
        c.pitch = 0
        c.roll = 0
        c.throttle = -32760
        c.throttleCRSF = intToCRSF(c.throttle)
    else:
        _thrustConstXZ = math.sqrt((c.a_z+G)**2 + c.a_x**2)
        c.ThrustXZ = MASS * _thrustConstXZ
        c.Theta = math.asin(c.a_x/_thrustConstXZ) * DEG_TO_RAD

        _thrustConstYZ = math.sqrt((c.a_z+G)**2 + c.a_y**2)
        c.ThrustYZ = MASS * _thrustConstYZ
        c.Phi = math.asin(c.a_y/_thrustConstYZ) * DEG_TO_RAD

        c.Thrust = MASS * math.sqrt(c.a_x**2 + c.a_y**2 + (c.a_z+G)**2)

        c.rpm = ThrustToRPM(c.Thrust/4) # Thrust per motor
        c.throttleCRSF = RPMtoThrottleCRSF(c.rpm)
        c.throttle = CRSFtoInt(c.throttleCRSF)

        c.yaw = degPerSecToInt(c.w_y)
        c.pitch = degPerSecToInt(c.Theta)
        c.roll = degPerSecToInt(c.Phi)
        # We are flying in ANGLE mode, which means that the stick
        # position is the pitch of the drone, not the acceleration
        # of the pitch. Thus, degrees per second is not actually
        # degrees per second, but rather just degrees.
        # The same applies to roll.

    passValues(c.yaw, c.pitch, c.roll, c.throttle, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0)

def Hover():
    c.forward = 0
    c.sideways = 0
    c.vertical = 0
    c.angle = 0

def flyForward():
    c.forward = 1
    c.sideways = 0
    c.vertical = 0
    c.angle = 0

def followHoops():
    contour, center = findContour(c.image, RED, GREEN, BLUE)
    if center is None:
        return
    else:
        dx = remap_range(center[1], 0, 640, -32768, 32767) # Center is (y, x)
        dy = remap_range(center[0], 0, 480, 32767, -32768) # dy is positive when the hoop is above the center
        print("dx: ", dx, "dy: ", dy)

    # Display the c.image
    cv.imshow("frame", c.image)
    print("Center: ", center)

def followTarget(*colors):
    contour, center = findContour(c.image, *colors)

    if center is None:
        return
    else:
        dx = remap_range(center[1], 0, 640, -1, 1, True)
        dy = remap_range(center[0], 0, 480, 1, -1, True)

        c.angle = dx
        c.vertical = dy
        c.forward = 0.25

        logging.debug("\n\n")
        logging.debug(f"\tAngle: {c.angle}")
        logging.debug(f"\tVertical: {c.vertical}\n\n")

def Stabilize():
    c.forward = 0
    c.sideways = 0
    c.vertical = 0
    c.angle = 0

    contour, center = findContour(c.image, BLUE)
    if center is None:
        return
    else:
        dx = remap_range(center[1], 0, 640, 1, -1, True)
        dy = remap_range(center[0], 0, 480, 1, -1, True)

        c.forward = 0
        c.sideways = dx
        c.vertical = dy
        c.angle = 0

        logging.debug("\n\n")
        logging.debug(f"\tSideways: {c.sideways}")
        logging.debug(f"\tVertical: {c.vertical}\n\n")
        logging.debug("\n\n")
