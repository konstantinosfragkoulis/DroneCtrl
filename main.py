import sys
import os
import cv2 as cv
import numpy as np

import mmap
import posix_ipc
import struct

import signal

import argparse
import logging

import time

from typing import *

from config import *
from config import Config as c
from utils import *
from conversions import *
from basic import *
from control import *
from takeoff import *


def Awake():
    """Initialization function that is called first even before Start()"""
    print("Program started")
    logging.debug("\tInitializing signal handlers...")

    for sig in signals_to_handle:
        signal.signal(sig, handle_signal)

    logging.debug("\tSignal handlers initialized")

def Start():
    """Initialization function that is called once when the program starts."""
    
    logging.debug("\tInitializing shared memory...")
    c.memory = posix_ipc.SharedMemory("/myshm")
    c.mapFile = mmap.mmap(c.memory.fd, c.memory.size)
    c.values = struct.unpack('i'*16, c.mapFile.read(64))
    
    logging.debug("\tShared memory initialized")
    if c.virtualCam:
        logging.debug("\tInitializing virtual camera...")
        
        size = 640 * 480 * 3

        c.virtCamMemory = posix_ipc.SharedMemory("/myVirtCamMem", posix_ipc.O_CREAT, size=size)
        c.virtCamMapFile = mmap.mmap(c.virtCamMemory.fd, size)
        c.image = np.zeros((480, 640, 3), dtype=np.uint8)

        logging.debug("\tVirtual camera initialized")
    else:
        logging.debug("\tInitializing camera...")
        
        c.cap = cv.VideoCapture("/dev/video0")
        
        logging.debug("\tCamera initialized")
    logging.debug("\tDisarming drone...")

    Disarm()
    c.state = State.Disarmed

    logging.debug("\tDrone disarmed")
    logging.debug("\tGetting first frame...")

    ret = False
    if c.virtualCam:
        getVirtualFrame()
        if c.image is not None:
            ret = True
    else:
        ret, _ = c.cap.read()
    if ret:
        logging.debug("\tFirst frame captured")
        logging.debug("\tArming drone...")
        Arm()
        c.state = State.Grounded
        logging.debug("\tDrone armed")
        CalculateTakeoff(TAKEOFF_HEIGHT, TAKEOFF_TIME)
        CalculateLanding()
        print("\nDrone initialized!\n\n\n")
    else:
        print("Failed to get first frame!\nThere is a problem with the camera")
        print("Cleaning up...")
        cleanup()
        return

def UpdateHelp():
    initialTime = time.perf_counter()
    while c.running:
        c.debugInfo = ""
        Update()

        c.dt = time.perf_counter() - initialTime
        initialTime = time.perf_counter()
    
    cleanup()

def Update():
    ret = False
    if c.virtualCam:
        getVirtualFrame()
        if c.image is not None:
            ret = True
    else:
        ret, c.image = c.cap.read()
    if not ret:
        log("Error: failed to capture image")
        c.running = False
        return
    else:
        keyPressed = cv.waitKey(1)


        if c.state == State.Disarmed:
            if keyPressed == ord('r'):
                try:
                    Arm()
                except Exception as e:
                    print(f"An error occurred while calling Arm() when Disarmed: {e}")
                    cleanup()
                c.state = State.Grounded
                log("Armed")
                
        elif c.state == State.Grounded:
            if keyPressed == ord('w'):
                c.state = State.TakingOff
                c.takeoffCnt = 0
                log("Taking off...")
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
                c.state = State.Disarmed
                log("Disarmed")
            else:
                try:
                    ZeroThrottle()
                except Exception as e:
                    print(f"An error occurred while calling ZeroThrottle() with the drone Grounded: {e}")
                    cleanup()

        elif c.state == State.TakingOff:
            if c.takeoffCnt < TAKEOFF_TIME1:
                try:
                    Takeoff(1)
                except Exception as e:
                    print(f"An error occurred while calling Takeoff(1): {e}")
                    cleanup()
            elif c.takeoffCnt < TAKEOFF_TIME:
                try:
                    Takeoff(2)
                except Exception as e:
                    print(f"An error occurred while calling Takeoff(2): {e}")
                    cleanup()
            else:
                c.state = State.Flying
                c.flyingState = FlyingState.Hovering
                c.takeoffCnt = 0
                log("Flying...")
                Hover()

            c.takeoffCnt += c.dt

        elif c.state == State.Flying:
            if keyPressed == ord('s'):
                c.state = State.Landing
                c.landingCnt = 0
                log("Landing...")
            elif keyPressed == ord('f'):
                c.flyingState = FlyingState.FollowingObject
                log("Following object")
            elif keyPressed == ord('h'):
                c.flyingState = FlyingState.Hovering
                log("Hovering")
            elif keyPressed == ord('w'):
                c.flyingState = FlyingState.FlyingForward
                log("Flying forward")
            elif keyPressed == ord('a'):
                c.flyingState = FlyingState.StabilizedHover
                c.stabilizedHoverTime = 0
                log("Stabilized hover")
            

            if c.flyingState == FlyingState.Hovering:
                log("Hovering")
                try:
                    Hover()
                except Exception as e:
                    print(f"An error occurred: {e}")
                    cleanup()
            elif c.flyingState == FlyingState.FollowingObject:
                try:
                    followTarget(BLUE)
                    pass
                except Exception as e:
                    print(f"An error occurred: {e}")
                    cleanup()
            elif c.flyingState == FlyingState.FlyingForward:
                try:
                    flyForward()
                except Exception as e:
                    print(f"An error occurred: {e}")
                    cleanup()
            elif c.flyingState == FlyingState.StabilizedHover:
                try:
                    Stabilize()
                    c.stabilizedHoverTime += c.dt
                except Exception as e:
                    print(f"An error occurred: {e}")
                    cleanup()

        elif c.state == State.Landing:
            if c.landingCnt < LANDING_TIME1:
                try:
                    Land(1)
                except Exception as e:
                    print(f"An error occurred while calling Land(1): {e}")
                    cleanup()
            elif c.landingCnt < LANDING_TIME:
                try:
                    Land(2)
                except Exception as e:
                    print(f"An error occurred while calling Land(2): {e}")
                    cleanup()
            else:
                ZeroThrottle()
                c.state = State.Grounded
                c.landingCnt = 0
                log("Landed")
            
            c.landingCnt += c.dt


        try:
            control()
        except Exception as e:
            print(f"An error occurred while calling control(): {e}")
            cleanup()


        if keyPressed == ord('q'):
            c.running = False
        
        cv.imshow("frame", c.image)
        printDebugInfo()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="A program that controls a drone and makes it autonomous."
    )
    parser.add_argument("-v", "--verbose", help="increase output verbosity", action="store_true")
    parser.add_argument("--virt", help="use virtual camera", action="store_true")
    args = parser.parse_args()
    if args.verbose:
        logging.basicConfig(level=logging.DEBUG)
    if args.virt:
        c.virtualCam = True

    logging.getLogger('PIL').setLevel(logging.WARNING)
    logging.debug("\tVerbose mode")
    Awake()
    Start()
    UpdateHelp()
