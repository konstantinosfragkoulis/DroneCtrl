import cv2 as cv
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
from init import *


def Awake():
    """Initialization function that is called first even before Start()"""
    print("Program started")
    initSignalHandlers()

def Start():
    """Initialization function that is called once when the program starts."""
    initSharedMemory()
    initCamera()
    
    logging.debug("\tDisarming drone...")

    Disarm()

    logging.debug("\tDrone disarmed")
    getFirstFrame()

def UpdateHelp():
    initialTime = time.perf_counter()
    while c.running:
        c.debugInfo = ""
        Update()

        currentTime = time.perf_counter()
        c.dt = currentTime - initialTime
        initialTime = currentTime
    
    cleanup()

def Update():
    ret = getFrame()
    if not ret:
        log("Error: failed to capture image")
        c.running = False
        return
    else:
        keyPressed = cv.waitKey(1)
        log(str(c.dt))


        if c.state == State.Disarmed:
            if keyPressed == ord('r'):
                Arm()
                c.state = State.Grounded
                log("Armed")
                
        elif c.state == State.Grounded:
            if keyPressed == ord('w'):
                c.state = State.TakingOff
                c.takeoffCnt = 0
                log("Taking off...")
            elif keyPressed == ord('r'):
                Disarm()
                log("Disarmed")
            else:
                ZeroThrottle()

        elif c.state == State.TakingOff:
            if c.takeoffCnt < TAKEOFF_TIME1:
                Takeoff(1)
            elif c.takeoffCnt < TAKEOFF_TIME:
                Takeoff(2)
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
                c.timer = 0
                log("Following object")
            elif keyPressed == ord('h'):
                c.flyingState = FlyingState.Hovering
                log("Hovering")
            elif keyPressed == ord('w'):
                c.flyingState = FlyingState.FlyingForward
                log("Flying forward")
            elif keyPressed == ord('a'):
                c.flyingState = FlyingState.StabilizedHover
                c.timer = 0
                log("Stabilized hover")
            elif keyPressed == ord('p'):
                c.flyingState = FlyingState.StabilizedHoverPreview
                c.timer = 0
                log("Stabilized hover preview")
            

            if c.flyingState == FlyingState.Hovering:
                log("Hovering")
                Hover()
            elif c.flyingState == FlyingState.FollowingObject:
                followTarget(BLUE)
                c.timer += c.dt
            elif c.flyingState == FlyingState.FlyingForward:
                flyForward()
            elif c.flyingState == FlyingState.StabilizedHover:
                Stabilize()
                c.timer += c.dt
            elif c.flyingState == FlyingState.StabilizedHoverPreview:
                Stabilize(True)
                c.timer += c.dt

        elif c.state == State.Landing:
            if c.landingCnt < LANDING_TIME1:
                Land(1)
            elif c.landingCnt < LANDING_TIME:
                Land(2)
            else:
                ZeroThrottle()
                c.state = State.Grounded
                c.landingCnt = 0
                log("Landed")
            
            c.landingCnt += c.dt


        control()


        if keyPressed == ord('q'):
            c.running = False
        
        cv.imshow("frame", c.image)
        printDebugInfo()


if __name__ == "__main__":
    initArgumentParser()
    Awake()
    Start()
    UpdateHelp()
