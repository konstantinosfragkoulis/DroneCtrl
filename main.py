import sys
import cv2 as cv
import numpy as np
import nptyping
import math

import mmap
import posix_ipc
import os
from time import sleep
from enum import IntEnum
import struct
import random

import signal

import argparse
import logging

import time

sys.path.insert(0, "/home/konstantinos/projects/drone/library")
import racecar_core
import racecar_utils as rc_utils

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

signals_to_handle = [
    signal.SIGINT,
    signal.SIGTERM,
    signal.SIGHUP,
    signal.SIGUSR1,
    signal.SIGUSR2,
    signal.SIGQUIT,
    signal.SIGABRT
]

PI = 3.14159
G = 9.81 # m/s^2

# Not implemented yet
MAXACCEL = 0.5 # 0.5 m/s^2
MAXTHRUST = 2.94 # ΣF = m * a => T - w = m * a => T = m * a + w =>
                 #T_max = 0.28 * 0.5 + 2.8 = 2.94 N
TRUEMAXTHRUST = 5.74 # MAXTHRUST + WEIGHT_N


#################################################################
###############  USER EDITABLE VARIABLES - START  ###############
#################################################################

TAKEOFF_TIME = 2 # The time it takes for the drone to take off
TAKEOFF_HEIGHT = 1 # The height the drone will take off to
WEIGHT = 0.8 # actual weight is 300 grams but Thrust(),
                # ThrustToRPM() and RPMtoThrottle() are not very
                # accurate

#################################################################
################  USER EDITABLE VARIABLES - END  ################
#################################################################



TAKEOFF_TIME1 = 2 * TAKEOFF_TIME / 3
TAKEOFF_TIME2 = TAKEOFF_TIME / 3
WEIGHT_N = WEIGHT * G # The weight of the drone in Newtons
HOVER_THRUST = WEIGHT_N # The thrust needed to hover
HOVER_THRUST_MOTOR = HOVER_THRUST / 4 # The thrust needed to hover per motor



PURPLE = ((130, 150, 150), (140, 255, 255))
ORANGE = ((10, 150, 150), (20, 255, 255))
RED = ((170, 125, 125), (10, 255, 255))
GREEN = ((55, 150, 150), (65, 255, 255))
BLUE = ((100, 125, 125), (115, 255, 255))

running = True # The main loop of the program

cap = None

memory = None
map_file = None
values = None

forward = 0 # Move front or back
angle = 0 # Turn left or right
vertical = 0 # Move up or down

height = 0 # The calculated height of the drone
speed = 0 # The calculated speed of the drone

image = None # The current image from the camera

dt = 0 # The time difference between to Update calls

takeoffCnt = 0
takeoffThrust1: float = 0
takeoffThrust2: float = 0
takeoffThrottle1: int = 0
takeoffThrottle2: int = 0


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
    # Release the camera
    cap.release()
    cv.destroyAllWindows()

    logging.debug("\tCamera released")
    logging.debug("\tClosing shared memory...")

    map_file.close()
    
    logging.debug("\tShared memory closed")
    print("Exiting...")
    sys.exit(0)

def handle_signal(signum, frame):
    """Handles many signals that stop program execution by cleaning up and exiting the program.
    VERY IMPORTANT! Under no circumstances should the program exit without calling cleanup or disarming the drone!"""
    print(f"\nDetected signal {signum}!\nCleaning up...")
    cleanup()

const = (PI * 1.225 * (0.0762**2) * 0.0635) / (360000 * 4) # * (RPM**2)
# Simplified thrust formula: T = (pi * rho * D^2 * n^2 * P) / 4
def Thrust(rpm: int):
    """Calculates the thrust produced by a motor spinning at a given RPM."""
    global const
    return const * (rpm ** 2)

const2 = math.sqrt((4 * 100 * 3600) / (PI * 1.225 * (0.0762 ** 2) * 0.0635)) # * sqrt(RPM)
def ThrustToRPM(thrust: float):
    """Calculates the RPM of a motor needed to produce the given thrust by it."""
    global const2
    return int((math.sqrt(thrust)) * const2)

# TODO: optimize constants which are calculated every time
def RPMtoThrottle(rpm: int):
    """Converts the RPM of a motor to a throttle value that can be passed to the shared memory."""
    return int(270782500 + (970.5814 - 270782500)/(1 + (rpm/10196720000)**1.011358))

def passValues(*inputs):
    """Passes the given values to the shared memory, effectively transmitting them to the drone."""
    global values
    if len(inputs) > 16:
        logging.debug("\tError: Too many inputs. Maximum is 16.")
        return
    values = list(inputs) + [0] * (15 - len(inputs))
    map_file.seek(0)  # Go back to the beginning of the mmap
    map_file.write(struct.pack('i'*16, *values))

def Arm():
    """Arms the drone by lowering the throttle to the minimum value
    and setting AUX1 to high."""
    passValues(0, 0, 0, -32760, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
    sleep(0.1)
    passValues(0, 0, 0, -32760, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0)

def Disarm():
    """Disarms the drone by lowering the throttle to the minimum value
    and setting AUX1 to low."""
    passValues(0, 0, 0, -32760, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)

def CalculateTakeoff(h: float, t: float):
    """Calculates the thrust needed for the drone to take off to a given
    height `h` in a given time `t`.""" 
    global height
    global speed
    global takeoffThrust1
    global takeoffThrust2
    global takeoffThrottle1
    global takeoffThrottle2

    a1 = (3 * h)/(t ** 2)
    takeoffThrust1 = WEIGHT * (a1 + G) / G # Divide by G to get the thrust in kg
    takeoffRPM1 = ThrustToRPM(takeoffThrust1/4) # Thrust per motor
    takeoffThrottle1 = RPMtoThrottle(takeoffRPM1)
    u1 = (2 * a1 * t)/3

    a2  = -3 * u1/t
    takeoffThrust2 = WEIGHT * (a2 + G) / G # Divide by G to get the thrust in kg
    takeoffRPM2 = ThrustToRPM(takeoffThrust2/4) # Thrust per motor
    takeoffThrottle2 = RPMtoThrottle(takeoffRPM2)

    print("a1: ", a1)
    print("u1: ", u1)
    print("T1: ", takeoffThrust1)
    print("RPM1: ", takeoffRPM1)
    print("Throttle1: ", takeoffThrottle1)
    print("Predicted thrust at takeoff 1: ", Thrust(takeoffRPM1) * 4)

    print("a2: ", a2)
    print("T2: ", takeoffThrust2)
    print("RPM2: ", takeoffRPM2)
    print("Throttle2: ", takeoffThrottle2)
    print("Predicted thrust at takeoff 2: ", Thrust(takeoffRPM2) * 4)

    print("\n")

def Takeoff(takeoffStage: int):
    if takeoffStage == 1:
        logging.debug(f"\tSetting throttle to {takeoffThrottle1} for takeoff stage 1...")
        passValues(0, 0, 0, takeoffThrottle1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0)
    elif takeoffStage == 2:
        logging.debug(f"\tSetting throttle to {takeoffThrottle2} for takeoff stage 2...")
        passValues(0, 0, 0, takeoffThrottle2, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0)
    else:
        logging.debug(f"\tInvalid takeoff stage {takeoffStage}. Must be 1 or 2")

def Hover():
    # TODO: Optimize: Don't calculate the RPM every time
    passValues(0, 0, 0, RPMtoThrottle(ThrustToRPM(HOVER_THRUST_MOTOR)), 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0)

def control(speed, angle, height):
    pass

def findContour(image, *colors):
    contours = []
    for color in colors:
        contour = rc_utils.get_largest_contour(rc_utils.find_contours(image, color[0], color[1]))
        if contour is not None:
            contours.append(contour)
            rc_utils.draw_contour(image, contour, color[1])
            center = rc_utils.get_contour_center(contour)
            rc_utils.draw_circle(image, center)
    if len(contours) > 0:
        largest_contour = max(contours, key=rc_utils.get_contour_area)
        largest_center = rc_utils.get_contour_center(largest_contour)
        return largest_contour, largest_center
    else:
        return None, None

def followHoops():
    global image
    global speed
    global angle
    global height
    
    contour, center = findContour(image, RED, GREEN, BLUE)
    if center is None:
        return
    else:
        dx = rc_utils.remap_range(center[1], 0, 640, -32768, 32767) # Center is (y, x)
        dy = rc_utils.remap_range(center[0], 0, 480, 32767, -32768) # dy is positive when the hoop is above the center
        print("dx: ", dx, "dy: ", dy)

    # Display the image
    cv.imshow("frame", image)
    print("Center: ", center)

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
    
    cap = cv.VideoCapture("/dev/video0")
    
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
    global dt
    global takeoffCnt
    global takeoffThrust1
    global takeoffThrust2
    global takeoffThrottle1
    global takeoffThrottle2

    ret, image = cap.read()
    if not ret:
        print("Error: failed to capture image")
        running = False
        return
    else:
        cv.imshow("frame", image)
        print("dt: ", dt)
        print(RPMtoThrottle(14000))
        print(Thrust(14000)*4)
        




        if state == State.Disarmed:
            if cv.waitKey(1) == ord('r'):
                Arm()
                state = State.Grounded
                
        elif state == State.Grounded:
            # print(rc_utils.remap_range(RPMtoThrottle(10000), -32768, 32767, 1000, 2000, True))
            # print(RPMtoThrottle(10000))
            if cv.waitKey(1) == ord('w'):
                state = State.TakingOff
                takeoffCnt = 0
                print("Taking off...")

        elif state == State.TakingOff:
            if takeoffThrust1 == 0 or takeoffThrust2 == 0:
                logging.debug("\tTakeoff thrust not calculated yet")
                logging.debug("\tCalculating takeoff thrust...")
                CalculateTakeoff(TAKEOFF_HEIGHT, TAKEOFF_TIME)
                logging.debug("\tTakeoff thrust calculated")
                logging.debug("\tStarting takeoff...\n")
            elif takeoffCnt < TAKEOFF_TIME1:
                # Takeoff
                Takeoff(1)
            elif takeoffCnt < TAKEOFF_TIME:
                # Takeoff
                Takeoff(2)
            else:
                state = State.Flying
                takeoffCnt = 0
                print("Flying...")
                Hover()

            takeoffCnt += dt

        elif state == State.Flying:
            # Fly
            pass

        elif state == State.Landing:
            # Land
            pass
            

        


        if cv.waitKey(1) == ord('q'):
            running = False


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="A program that controls a drone and makes it semi-autonomous."
    )
    parser.add_argument("-v", "--verbose", help="increase output verbosity", action="store_true")
    args = parser.parse_args()
    if args.verbose:
        logging.basicConfig(level=logging.DEBUG)

    logging.debug('Only shown in debug mode')
    Awake()
    Start()
    UpdateHelp()
