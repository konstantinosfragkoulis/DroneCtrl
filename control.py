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
            c.stablizedHoverData.centerY = center[0]
            c.stablizedHoverData.centerX = center[1]
        
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
            if c.stablizedHoverData.centerX > CAM_WIDTHD2:
                c.sideways = -STABILIZED_HOVER_STEP_ACCELERATIOND2
            elif c.stablizedHoverData.centerX < CAM_WIDTHD2:
                c.sideways = STABILIZED_HOVER_STEP_ACCELERATIOND2
            else:
                c.sideways = 0
            
            if c.stablizedHoverData.centerY > CAM_HEIGHTD2:
                c.vertical = -STABILIZED_HOVER_STEP_ACCELERATIOND2
            elif c.stablizedHoverData.centerY < CAM_HEIGHTD2:
                c.vertical = STABILIZED_HOVER_STEP_ACCELERATIOND2
            else:
                c.vertical = 0
            
            log("Sideways: ", c.sideways, " Vertical: ", c.vertical)
        elif c.stabilizedHoverTime < STABILIZED_HOVER_STEP_DURATION:
           
            if c.stablizedHoverData.centerX > CAM_WIDTHD2:
                c.sideways = STABILIZED_HOVER_STEP_ACCELERATIOND2
            elif c.stablizedHoverData.centerX < CAM_WIDTHD2:
                c.sideways = -STABILIZED_HOVER_STEP_ACCELERATIOND2
            else:
                c.sideways = 0
            
            if c.stablizedHoverData.centerY > CAM_HEIGHTD2:
                c.vertical = STABILIZED_HOVER_STEP_ACCELERATIOND2
            elif c.stablizedHoverData.centerY < CAM_HEIGHTD2:
                c.vertical = -STABILIZED_HOVER_STEP_ACCELERATIOND2
            else:
                c.vertical = 0
        elif not c.stablizedHoverData.adjustedHoverData:
            c.stablizedHoverData.dCenterY = abs(c.stablizedHoverData.centerY - center[0])
            c.stablizedHoverData.dCenterX = abs(c.stablizedHoverData.centerX - center[1])
            c.stablizedHoverData.adjustedHoverData = True

            # In STABILIZED_HOVER_STEP_DURATION seconds moved dCenterX and dCenterY
            # So, to move abs(center[0] - WIDTHD2) with STABILIZED_HOVER_STEP_ACCELERATIOND2 we need c.stabilizationDuration seconds
            c.stablizedHoverData.stabilizationDurationY = abs(CAM_HEIGHTD2 - center[0])*STABILIZED_HOVER_STEP_DURATION/c.stablizedHoverData.dCenterY
            c.stablizedHoverData.stabilizationDurationX = abs(CAM_WIDTHD2 - center[1])*STABILIZED_HOVER_STEP_DURATION/c.stablizedHoverData.dCenterX

            c.stablizedHoverData.stabilizationDurationYD2 = c.stablizedHoverData.stabilizationDurationY/2
            c.stablizedHoverData.stabilizationDurationXD2 = c.stablizedHoverData.stabilizationDurationX/2

            c.stablizedHoverData.left = c.stablizedHoverData.centerX < CAM_WIDTHD2
            if c.stablizedHoverData.centerX == CAM_WIDTHD2:
                c.stablizedHoverData.left = None
            
            c.stablizedHoverData.up = c.stablizedHoverData.centerY < CAM_HEIGHTD2
            if c.stablizedHoverData.centerY == CAM_HEIGHTD2:
                c.stablizedHoverData.up = None

            c.stablizedHoverData.stabilizationDuration = max(c.stablizedHoverData.stabilizationDurationX, c.stablizedHoverData.stabilizationDurationY)

        elif c.stabilizedHoverTime < c.stablizedHoverData.stabilizationDuration:

            if c.stabilizedHoverTime < c.stablizedHoverData.stabilizationDurationYD2:
                if c.stablizedHoverData.up is None:
                    c.vertical = 0
                elif c.stablizedHoverData.up:
                    c.vertical = STABILIZED_HOVER_STEP_ACCELERATIOND2
                elif not c.stablizedHoverData.up:
                    c.vertical = -STABILIZED_HOVER_STEP_ACCELERATIOND2
            elif c.stabilizedHoverTime < c.stablizedHoverData.stabilizationDurationY:
                if c.stablizedHoverData.up is None:
                    c.vertical = 0
                elif c.stablizedHoverData.up:
                    c.vertical = -STABILIZED_HOVER_STEP_ACCELERATIOND2
                elif not c.stablizedHoverData.up:
                    c.vertical = STABILIZED_HOVER_STEP_ACCELERATIOND2

            if c.stabilizedHoverTime < c.stablizedHoverData.stabilizationDurationXD2:
                if c.stablizedHoverData.left is None:
                    c.sideways = 0
                elif c.stablizedHoverData.left:
                    c.sideways = STABILIZED_HOVER_STEP_ACCELERATIOND2
                elif not c.stablizedHoverData.left:
                    c.sideways = -STABILIZED_HOVER_STEP_ACCELERATIOND2
            elif c.stabilizedHoverTime < c.stablizedHoverData.stabilizationDurationX:
                if c.stablizedHoverData.left is None:
                    c.sideways = 0
                elif c.stablizedHoverData.left:
                    c.sideways = -STABILIZED_HOVER_STEP_ACCELERATIOND2
                elif not c.stablizedHoverData.left:
                    c.sideways = STABILIZED_HOVER_STEP_ACCELERATIOND2
        else:
            log("Drone Stabilized")
            c.stabilizedHoverTime = 0
            c.stablizedHoverData = StablizedHoverData(0, 0, 0, 0, 0, 0, False, 999, 999, 999, True, True)

        c.angle = 0
        
        log(f"\nCenter: {center}")
        log(f"Vertical: {c.vertical}")
        log("DX: ", str(center[1]), " DY: " + str(center[0]))
        log(f"Sideways: {c.sideways}, Vertical: {c.vertical}")
