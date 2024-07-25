from basic import safeCall, log, cleanup, passValues
from config import *
from config import Config as c
from conversions import *
from utils import *
import cv2 as cv

@safeCall
def ZeroThrottle():
    """Sets the throttle to the minimum value, with the drone still armed."""
    c.forward = 0
    c.angle = 0
    c.vertical = -10

@safeCall
def applyRotation():
    """Rotate the drone to the desired angles."""
    if abs(c.Theta - c.fd.pitch) > 0.1:
        tmp = (c.Theta - c.fd.pitch) / TIME_TO_ROTATE
        log("Pitch speed: ", str(tmp))
        c.pitch = degPerSecToInt(tmp)
        c.fd.pitch += tmp * c.dt
        log("Current pitch: ", str(c.fd.pitch))
    else:
        c.pitch = 0

    if abs(c.Phi - c.fd.roll) > 0.1:
        tmp = (c.Phi - c.fd.roll) / TIME_TO_ROTATE
        log("Roll speed: ", str(tmp))
        c.roll = degPerSecToInt(tmp)
        c.fd.roll += tmp * c.dt
        log("Current roll: ", str(c.fd.roll))
    else:
        c.roll = 0

    if abs(c.w_y - c.fd.yaw) > 0.1:
        tmp = (c.w_y - c.fd.yaw) / TIME_TO_ROTATE
        log("Yaw speed: ", str(tmp))
        c.yaw = degPerSecToInt(tmp)
        c.fd.yaw += tmp * c.dt
        log("Current yaw: ", str(c.fd.yaw))
    else:
        c.yaw = 0
        

@safeCall
def control():
    """Convert the forward, angle, vertical, and sideways values to pitch,
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
    if c.virtual:
        c.a_z = (c.vertical+VERTICAL_ACCELERATION_OFFSET_SIM)*MAX_VERTICAL_ACCELERATION
    else:
        c.a_z = (c.vertical+VERTICAL_ACCELERATION_OFFSET)*MAX_VERTICAL_ACCELERATION
    c.w_y = c.angle*MAX_ANGULAR_ACCELERATION

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

        applyRotation()

        # c.yaw = degPerSecToInt(c.w_y)
        # c.pitch = degPerSecToInt(c.Theta)
        # c.roll = degPerSecToInt(c.Phi)


    log(f"Yaw: {c.yaw}, Pitch: {c.pitch}, Roll: {c.roll}, Throttle: {c.throttle}")
    passValues(c.yaw, c.pitch, c.roll, c.throttle, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0)

@safeCall
def Hover():
    c.forward = 0
    c.sideways = 0
    c.vertical = 0
    c.angle = 0

@safeCall
def flyForward():
    c.forward = 0.5
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
def _getAngle(x):
    return remap_range(x, 0, 640, -60, 60)/MAX_ANGULAR_ACCELERATION

@safeCall
def followTarget(*colors):
    """Follow the biggest object of the specified color(s)."""
    c.forward = 0.25
    c.sideways = 0
    c.vertical = 0
    c.angle = 0

    _, center = findContour(c.image, *colors)
    if center is None:
        return
    else:
        c.sd.accelW = _getAngle(center[1])
        if c.timer == 0:
            c.sd.accelZ = _getAccel(center[0], CAM_HEIGHTD2, STABILIZED_HOVER_STEP_ACCELERATION_ZD2)
        elif c.timer < STABILIZED_HOVER_STEP_DURATIOND2:
            c.vertical = c.sd.accelZ
            c.angle = c.sd.accelW
        elif c.timer < STABILIZED_HOVER_STEP_DURATION:
            c.vertical = -c.sd.accelZ
            c.angle = c.sd.accelW
        else:
            c.timer = 0.0001
            c.sd.accelZ = _getAccel(center[0], CAM_HEIGHTD2, STABILIZED_HOVER_STEP_ACCELERATION_ZD2)
            c.sd.accelW = remap_range(center[1], 0, 640, -60, 60)/MAX_ANGULAR_ACCELERATION
            log(f"New accelZ: {c.sd.accelZ}, New accelW: {c.sd.accelW}")

        log("\n\n")
        log(f"\tAngle: {c.angle}")
        log(f"\tVertical: {c.vertical}\n\n")

@safeCall
def _getAccel(center, mid, accel):
    """Get the acceleration needed to align the drone with the point
    of reference It is used in the Stabilize() function that does not
    provide smooth movement."""
    if abs(center - mid) < STABILIZED_HOVER_DEADZONE:
        return 0
    elif center > mid:
        return -accel
    else:
        return accel

@safeCall
def Stabilize(preview=False):
    """Align the drone with the point of reference and stabilize it.
    The movement is *not* smooth."""
    c.forward = 0
    c.sideways = 0
    c.vertical = 0
    c.angle = 0

    _, center = findContour(c.image, BLUE)
    if center is None:
        return
    else:
        if c.timer == 0:
            c.sd.accelZ = _getAccel(center[0], CAM_HEIGHTD2, STABILIZED_HOVER_STEP_ACCELERATION_ZD2)
            c.sd.accelX = _getAccel(center[1], CAM_WIDTHD2, STABILIZED_HOVER_STEP_ACCELERATION_YD2)
        elif c.timer < STABILIZED_HOVER_STEP_DURATIOND2:
            if not preview:
                c.sideways = c.sd.accelX
                c.vertical = c.sd.accelZ
        elif c.timer < STABILIZED_HOVER_STEP_DURATION:
            if not preview:
                c.sideways = -c.sd.accelX
                c.vertical = -c.sd.accelZ
        else:
            c.timer = 0.0001
            c.sd.accelZ = _getAccel(center[0], CAM_HEIGHTD2, STABILIZED_HOVER_STEP_ACCELERATION_ZD2)
            c.sd.accelX = _getAccel(center[1], CAM_WIDTHD2, STABILIZED_HOVER_STEP_ACCELERATION_YD2)
            log(f"New accelX: {c.sd.accelX}, New accelZ: {c.sd.accelZ}")
        
        log(f"accelX: {c.sd.accelX}, accelZ: {c.sd.accelZ}")
        log(f"Sideways: {c.sideways}, Vertical: {c.vertical}")
        log(f"Center: {center}")
        log(f"hoverTime: {c.timer}")
