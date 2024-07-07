import sys
import csv
import cv2 as cv
import struct
import logging
from time import sleep
from datetime import datetime
from config import *
from config import Config as c
from tabulate import tabulate

def safeCall(func):
    def wrappedFunc(*args, **kwargs):
        try:
            return func(*args, **kwargs)
        except Exception as e:
            print(f"An error occurred while calling {func.__name__}(): {e}")
            cleanup()
    return wrappedFunc

def cleanup():
    c.running = False
    c.forward = 0
    c.sideways = 0
    c.vertical = -10
    c.angle = 0


    print("Program stopped running")
    logging.debug("\tDisarming...")

    from control import control
    control()
    sleep(0.2)

    c.state = State.Disarmed
    Disarm()

    logging.debug("\tDrone disarmed")
    if not c.virtual:
        logging.debug("\tReleasing camera...")
        c.cap.release()
        cv.destroyAllWindows()
        logging.debug("\tCamera released")

    logging.debug("\tClosing shared memory...")

    c.mapFile.close()
    
    logging.debug("\tShared memory closed")
    print("Exiting...")
    sys.exit(0)

@safeCall
def handle_signal(signum, frame):
    """Handles many signals that stop program execution by cleaning
    up and exiting the program.\nVERY IMPORTANT! Under no circumstances
    should the program exit without calling cleanup or disarming the
    drone!"""
    print(f"\nDetected signal {signum}!\nCleaning up...")
    cleanup()

@safeCall
def passValues(*inputs):
    """Passes the given values to the shared memory, effectively transmitting them to the drone."""
    if len(inputs) != 16:
        logging.debug("\tError: You must pass 16 values!")
        cleanup()
        return
    c.values = list(inputs) + [0] * (16 - len(inputs))
    c.mapFile.seek(0)
    c.mapFile.write(struct.pack('i'*16, *c.values))

@safeCall
def Arm():
    """Arms the drone by lowering the throttle to the minimum value
    and setting AUX1 to high."""
    passValues(0, 0, 0, -32768, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
    sleep(0.5)
    passValues(0, 0, 0, -32768, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0)

@safeCall
def Disarm():
    """Disarms the drone by lowering the throttle to the minimum value
    and setting AUX1 to low."""
    passValues(0, 0, 0, -32768, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)

@safeCall
def ZeroThrottle():
    """Sets the throttle to the minimum value, with the drone still armed."""
    c.forward = 0
    c.angle = 0
    c.vertical = -10

@safeCall
def log(*strings):
    """Print debug info."""
    strings = [str(element) for element in strings]
    for string in strings:
        c.debugInfo += string
    c.debugInfo += "\n"

@safeCall
def printDebugInfo():
    """Prints debug information when the -v flag is passed to the program."""
    from conversions import intToCRSF, intToDegPerSec
    headers = ["States", "", "CRSF", "int16", "Angle", "", "Accelerations"]
    data = [
        {"State": c.state.name,       "1": "Yaw",      "CRSF": intToCRSF(c.yaw),   "int16": c.yaw,      "Angle": round(intToDegPerSec(c.w_y), 4), "2": "x:", "Accelerations": c.a_x},
        {"State": c.flyingState.name, "1": "Pitch",    "CRSF": intToCRSF(c.pitch), "int16": c.pitch,    "Angle": round(c.Theta, 4),               "2": "y:", "Accelerations": c.a_y},
        {"State": "",                 "1": "Roll",     "CRSF": intToCRSF(c.roll),  "int16": c.roll,     "Angle": round(c.Phi, 4),                 "2": "z:", "Accelerations": c.a_z},
        {"State": "",                 "1": "Throttle", "CRSF": c.throttleCRSF,     "int16": c.throttle, "Angle": round(c.angle, 4),               "2": "w:", "Accelerations": c.w_y},
    ]

    table_data = [[item["State"], item["1"], item["CRSF"], item["int16"], item["Angle"], item["2"], item["Accelerations"]] for item in data]

    print("\033c", end="")

    print(tabulate(table_data, headers=headers, tablefmt="fancy_grid", numalign="left", stralign="right"))

    print(c.debugInfo)

    headers = ["Timestamp", "State", "Flying State", "Yaw int16", "Pitch int16", "Pitch Angle", "Roll int16", "Roll Angle", "Throttle int16", "Acc X", "Acc Y", "Acc Z", "W Y", "Thrust"]
    data = [
        datetime.now().isoformat(),
        c.state.name,
        c.flyingState.name,
        c.yaw,
        c.pitch,
        round(c.Theta, 4),
        c.roll,
        round(c.Phi, 4),
        c.throttle,
        c.a_x,
        c.a_y,
        c.a_z,
        c.w_y,
        c.Thrust
    ]

    writtenHeaders = True
    if c.startTime is None:
        c.startTime = datetime.now()
        writtenHeaders = False
    with open(f"./logs/log{c.startTime}.csv", "a", newline='') as csvfile:
        writer = csv.writer(csvfile)
        if not writtenHeaders:
            writer.writerow(headers)
            writtenHeaders = True
        writer.writerow(data)
