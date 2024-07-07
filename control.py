import logging
from basic import cleanup, passValues, log
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
        log("\tControl() called while disarmed")
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

    c.ThrustXZ = 0
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
        c.Theta = math.atan(c.a_x/(c.a_z+G)) * DEG_TO_RAD

        c.Phi = math.atan(c.a_y/(c.a_z+G)) * DEG_TO_RAD

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

def _getAccel(center, mid, accel):
    if center == mid:
        return 0
    elif center > mid:
        return -accel
    else:
        return accel

def Stabilize():
    c.forward = 0
    c.sideways = 0
    c.vertical = 0
    c.angle = 0

    contour, center = findContour(c.image, BLUE)
    if center is None:
        return
    else:
        if c.stabilizedHoverTime == 0:
            c.hd.centerY = center[0]
            c.hd.centerX = center[1]

            c.hd.accelY = _getAccel(c.hd.centerY, CAM_HEIGHTD2, STABILIZED_HOVER_STEP_ACCELERATIOND2)
            c.hd.accelX = _getAccel(c.hd.centerX, CAM_WIDTHD2, STABILIZED_HOVER_STEP_ACCELERATIOND2)
        
        # For the first second, the drone will move a bit left/right and
        # up/down based on the center of the reference object.

        # Then, we will use the change of the center of the reference object
        # to determine how much the drone should move left/right and up/down.

        # So basically, the drone will move in steps. Since we only know
        # the position of the drone relative to the reference object, we
        # will first calculate how big one step is, and then perform all
        # the steps at once to reach the desired position.
        
        # Move just one step
        c.forward = 0

        if c.stabilizedHoverTime < STABILIZED_HOVER_STEP_DURATIOND2:
            c.sideways = c.hd.accelX
            c.vertical = c.hd.accelY
            
            log("Sideways: ", c.sideways, " Vertical: ", c.vertical)
        elif c.stabilizedHoverTime < STABILIZED_HOVER_STEP_DURATION:
            c.sideways = -c.hd.accelX
            c.vertical = -c.hd.accelY
            
            log("Sideways: ", c.sideways, " Vertical: ", c.vertical)
            log(c.hd.adjustedHoverData)
        elif not c.hd.adjustedHoverData:
            c.hd.dCenterY = abs(CAM_HEIGHTD2 - center[0])
            c.hd.dCenterX = abs(CAM_WIDTHD2 - center[1])
            c.hd.adjustedHoverData = True

            dy = c.hd.accelY * (STABILIZED_HOVER_STEP_DURATIOND2**2)
            dx = c.hd.accelX * (STABILIZED_HOVER_STEP_DURATIOND2**2)

            dyPx = abs(c.hd.centerY - center[0])
            dxPx = abs(c.hd.centerX - center[1])

            print("dyPx: ", dyPx, " dxPx: ", dxPx)
            print("dy: ", dy, " dx: ", dx)
            print("dCenterY: ", c.hd.dCenterY, " dCenterX: ", c.hd.dCenterX)

            if dy == 0 or dyPx == 0:
                c.hd.h = 0
            else:
                c.hd.h = (c.hd.dCenterY * dy) / dyPx
            
            if dx == 0 or dxPx == 0:
                c.hd.d = 0
            else:
                c.hd.d = (c.hd.dCenterX * dx) / dxPx

            c.hd.stabilizationDuration = PI + STABILIZED_HOVER_STEP_DURATION

            print("d: ", c.hd.d, " h: ", c.hd.h)

        elif c.stabilizedHoverTime < c.hd.stabilizationDuration:
            log(c.hd.adjustedHoverData)

            c.sideways = math.sin(c.stabilizedHoverTime-STABILIZED_HOVER_STEP_DURATION + 3*PI/2) * c.hd.d/(2*MAX_SIDEWAYS_ACCELERATION)
            c.vertical = math.sin(c.stabilizedHoverTime-STABILIZED_HOVER_STEP_DURATION + 3*PI/2) * c.hd.h/(2*MAX_VERTICAL_ACCELERATION)
            # c.hd.d/2 because the format is A*sin(Bx + C) where A is the amplitude
            # and the amplitude is half of the distance the drone should.

        else:
            log("Drone Stabilized")
            #c.stabilizedHoverTime = 0
            #c.hd = StablizedHoverData(0, 0, 0, 0, False, 0, 0, 0, 0, 0)

        c.angle = 0
        
        log(f"\nCenter: {center}")
        log(f"Vertical: {c.vertical}")
        log("DX: ", str(center[1]), " DY: " + str(center[0]))
        log(f"Sideways: {c.sideways}, Vertical: {c.vertical}")
