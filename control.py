import logging
from basic import cleanup, passValues, log
from config import *
from config import Config as c
from conversions import *
from utils import *
from basic import safeCall
import cv2 as cv

@safeCall
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

@safeCall
def Hover():
    c.forward = 0
    c.sideways = 0
    c.vertical = 0
    c.angle = 0

@safeCall
def flyForward():
    c.forward = 1
    c.sideways = 0
    c.vertical = 0
    c.angle = 0

@safeCall
def followHoops():
    contour, center = findContour(c.image, RED, GREEN, BLUE)
    if center is None:
        return
    else:
        dx = remap_range(center[1], 0, 640, -32768, 32767) # Center is (y, x)
        dy = remap_range(center[0], 0, 480, 32767, -32768) # dy is positive when the hoop is above the center
        print("dx: ", dx, "dy: ", dy)

    # Display the image
    cv.imshow("frame", c.image)
    print("Center: ", center)

@safeCall
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

@safeCall
def _getAccel2(center, mid, accel):
    if center == mid:
        return 0
    elif center > mid:
        return -accel
    else:
        return accel

@safeCall
def _getAccel(center, mid, accel):
    if abs(center - mid) < STABILIZED_HOVER_DEADZONE:
        return 0
    elif center > mid:
        return -accel
    else:
        return accel

@safeCall
def Stabilize2():
    c.forward = 0
    c.sideways = 0
    c.vertical = 0
    c.angle = 0

    contour, center = findContour(c.image, BLUE)
    if center is None:
        return
    else:
        if c.stabilizedHoverTime == 0:
            c.hd2.centerY = center[0]
            c.hd2.centerX = center[1]

            c.hd2.accelY = _getAccel2(c.hd2.centerY, CAM_HEIGHTD2, STABILIZED_HOVER_STEP_ACCELERATION_ZD2)
            c.hd2.accelX = _getAccel2(c.hd2.centerX, CAM_WIDTHD2, STABILIZED_HOVER_STEP_ACCELERATION_YD2)

        if c.stabilizedHoverTime < STABILIZED_HOVER_STEP_DURATIOND2:
            c.sideways = c.hd2.accelX
            c.vertical = c.hd2.accelY
        elif c.stabilizedHoverTime < STABILIZED_HOVER_STEP_DURATION:
            c.sideways = -c.hd2.accelX
            c.vertical = -c.hd2.accelY
            log(c.hd2.adjustedHoverData)
        elif not c.hd2.adjustedHoverData:
            c.hd2.dCenterY = abs(CAM_HEIGHTD2 - center[0])
            c.hd2.dCenterX = abs(CAM_WIDTHD2 - center[1])
            c.hd2.adjustedHoverData = True

            c.hd2.dy = (c.hd2.accelY*MAX_VERTICAL_ACCELERATION) * (STABILIZED_HOVER_STEP_DURATIOND2**2)
            c.hd2.dx = (c.hd2.accelX*MAX_SIDEWAYS_ACCELERATION) * (STABILIZED_HOVER_STEP_DURATIOND2**2)

            print(f"accelY: {c.hd2.accelY}, accelX: {c.hd2.accelX}")
            print(f"Actual accelY: {c.hd2.accelY*MAX_VERTICAL_ACCELERATION}, Actual accelX: {c.hd2.accelX*MAX_SIDEWAYS_ACCELERATION}")
            print(f"duration: {STABILIZED_HOVER_STEP_DURATION}")
            print("dx: ", c.hd2.dx, " dy: ", c.hd2.dy)

            c.hd2.dyPx = abs(c.hd2.centerY - center[0])
            c.hd2.dxPx = abs(c.hd2.centerX - center[1])

            print("dyPx: ", c.hd2.dyPx, " dxPx: ", c.hd2.dxPx)
            print("dCenterY: ", c.hd2.dCenterY, " dCenterX: ", c.hd2.dCenterX)

            if c.hd2.dy == 0 or c.hd2.dyPx == 0:
                c.hd2.h = 0
            else:
                c.hd2.h = ((c.hd2.dCenterY * c.hd2.dy) / c.hd2.dyPx) * (-1 if c.hd2.accelY < 0 else 1)
            
            if c.hd2.dx == 0 or c.hd2.dxPx == 0:
                c.hd2.d = 0
            else:
                c.hd2.d = ((c.hd2.dCenterX * c.hd2.dx) / c.hd2.dxPx) * (-1 if c.hd2.accelX < 0 else 1)

            c.hd2.stabilizationDuration = PI + STABILIZED_HOVER_STEP_DURATION

            print("d: ", c.hd2.d, " h: ", c.hd2.h)

        elif c.stabilizedHoverTime < c.hd2.stabilizationDuration:
            log(c.hd2.adjustedHoverData)

            c.sideways = math.sin(c.stabilizedHoverTime-STABILIZED_HOVER_STEP_DURATION + 3*PI/2) * c.hd2.d/(2*MAX_SIDEWAYS_ACCELERATION)
            c.vertical = -math.sin(c.stabilizedHoverTime-STABILIZED_HOVER_STEP_DURATION + 3*PI/2) * c.hd2.h/(2*MAX_VERTICAL_ACCELERATION)

        else:
            log("Drone Stabilized")
            #c.stabilizedHoverTime = 0
            #c.hd2 = StablizedHoverData2(0, 0, 0, 0, False, 0, 0, 0, 0, 0)

        c.angle = 0
        
        log(f"\nCenter: {center}")
        log(f"Vertical: {c.vertical}")
        log(f"Sideways: {c.sideways}, Vertical: {c.vertical}")
        log(f"h: {c.hd2.h}, d: {c.hd2.d}")
        log(f"dx: {c.hd2.dx}, dy: {c.hd2.dy} dxPx: {c.hd2.dxPx}, dyPx: {c.hd2.dyPx}")
        log(f"AccelX: {c.hd2.accelX}, AccelY: {c.hd2.accelY}")

@safeCall
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
            c.hd.accelY = _getAccel(center[0], CAM_HEIGHTD2, STABILIZED_HOVER_STEP_ACCELERATION_ZD2)
            c.hd.accelX = _getAccel(center[1], CAM_WIDTHD2, STABILIZED_HOVER_STEP_ACCELERATION_YD2)
        elif c.stabilizedHoverTime < STABILIZED_HOVER_STEP_DURATIOND2:
            c.sideways = c.hd.accelX
            c.vertical = c.hd.accelY
        elif c.stabilizedHoverTime < STABILIZED_HOVER_STEP_DURATION:
            c.sideways = -c.hd.accelX
            c.vertical = -c.hd.accelY
        else:
            c.stabilizedHoverTime = 0.0001
            c.hd.accelY = _getAccel(center[0], CAM_HEIGHTD2, STABILIZED_HOVER_STEP_ACCELERATION_ZD2)
            c.hd.accelX = _getAccel(center[1], CAM_WIDTHD2, STABILIZED_HOVER_STEP_ACCELERATION_YD2)
            log(f"New accelX: {c.hd.accelX}, New accelY: {c.hd.accelY}")
        
        log(f"accelX: {c.hd.accelX}, accelY: {c.hd.accelY}")
        log(f"Sideways: {c.sideways}, Vertical: {c.vertical}")
        log(f"Center: {center}")
        log(f"hoverTime: {c.stabilizedHoverTime}")
