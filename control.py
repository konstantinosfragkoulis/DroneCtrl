import logging
from basic import cleanup, passValues
from config import *
from config import Config as c
from conversions import *
from utils import *
import cv2 as cv

def control():
    """Convert the forward, angle, and vertical values to pitch,
    yaw, roll and throttle values and pass them to the drone."""
    
    if c.forward > 1 or c.forward < -1:
        print(f"\tInvalid forward value {c.forward}. Must be between -1 and 1.")
        cleanup()
    elif c.angle > 1 or c.angle < -1:
        print(f"\tInvalid angle value {c.angle}. Must be between -1 and 1.")
        cleanup()
    elif (c.vertical > 1 or c.vertical < -1) and not c.vertical == -10:
        print(f"\tInvalid vertical value {c.vertical}. Must be between -1 and 1.")
        cleanup()
    elif c.sideways > 1 or c.sideways < -1:
        print(f"\tInvalid sideways value {c.sideways}. Must be between -1 and 1.")
        cleanup()

    a_x = c.forward*MAX_FORWARD_ACCELERATION
    a_y = c.sideways*MAX_SIDEWAYS_ACCELERATION
    a_z = (c.vertical+VERTICAL_ACCELERATION_OFFSET)*MAX_VERTICAL_ACCELERATION
    w_y = c.angle*MAX_ANGULAR_ACCELERATION

    logging.debug(f"\ta_x: , {a_x}, a_y: {a_y}, a_z: , {a_z}, w_y: , {w_y}")

    _thrustConstXZ = 0
    ThrustXZ = 0
    _thrustConstYZ = 0
    ThrustYZ = 0
    Theta = 0
    Phi = 0

    if c.vertical == -10:
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
    logging.debug(f"\tPitch c.angle: {Theta}")
    logging.debug(f"\tRoll c.angle: {Phi}")
    logging.debug(f"\tPitch: {pitch}")
    logging.debug(f"\tYaw: {yaw}")
    logging.debug(f"\tRoll: {roll}")

    passValues(yaw, pitch, roll, throttle, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0)

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
        dx = remap_range(center[1], 0, 640, -1, 1, True)
        dy = remap_range(center[0], 0, 480, 1, -1, True)

        c.forward = 0
        c.sideways = dx
        c.vertical = dy
        c.angle = 0

        logging.debug("\n\n")
        logging.debug(f"\tSideways: {c.sideways}")
        logging.debug(f"\tVertical: {c.vertical}\n\n")
        logging.debug("\n\n")
