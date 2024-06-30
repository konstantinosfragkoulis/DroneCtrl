import sys
import os
import csv
import cv2 as cv
import struct
import logging
from time import sleep
from datetime import datetime
from config import *
from config import Config as c
from tabulate import tabulate
from conversions import *

def cleanup():
    c.running = False
    print("Program stopped running")
    logging.debug("\tDisarming...")

    Disarm()

    logging.debug("\tDrone disarmed")
    logging.debug("\tReleasing camera...")
    c.cap.release()
    cv.destroyAllWindows()

    logging.debug("\tCamera released")
    logging.debug("\tClosing shared memory...")

    c.map_file.close()
    
    logging.debug("\tShared memory closed")
    print("Exiting...")
    sys.exit(0)

def handle_signal(signum, frame):
    """Handles many signals that stop program execution by cleaning
    up and exiting the program.\nVERY IMPORTANT! Under no circumstances
    should the program exit without calling cleanup or disarming the
    drone!"""
    print(f"\nDetected signal {signum}!\nCleaning up...")
    cleanup()

def passValues(*inputs):
    """Passes the given values to the shared memory, effectively transmitting them to the drone."""
    if len(inputs) != 16:
        logging.debug("\tError: You must pass 16 values!")
        cleanup()
        return
    c.values = list(inputs) + [0] * (16 - len(inputs))
    c.map_file.seek(0)  # Go back to the beginning of the mmap
    c.map_file.write(struct.pack('i'*16, *c.values))

def Arm():
    """Arms the drone by lowering the throttle to the minimum value
    and setting AUX1 to high."""
    passValues(0, 0, 0, -32768, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)
    sleep(0.5)
    passValues(0, 0, 0, -32768, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0)

def Disarm():
    """Disarms the drone by lowering the throttle to the minimum value
    and setting AUX1 to low."""
    passValues(0, 0, 0, -32768, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)

def ZeroThrottle():
    """Sets the throttle to the minimum value, with the drone still armed."""
    c.forward = 0
    c.angle = 0
    c.vertical = -10

def printDebugInfo():
    """Prints debug information when the -v flag is passed to the program."""
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

    headers = ["Timestamp", "State", "Yaw CRSF", "Yaw int16", "Yaw Angle", "Pitch CRSF", "Pitch int16", "Pitch Angle", "Roll CRSF", "Roll int16", "Roll Angle", "Throttle CRSF", "Throttle int16", "Acc X", "Acc Y", "Acc Z", "W Y", "Thrust"]
    data = [
        datetime.now().isoformat(),
        c.state.name,
        intToCRSF(c.yaw),
        c.yaw,
        round(intToDegPerSec(c.w_y), 4),
        intToCRSF(c.pitch),
        c.pitch,
        round(c.Theta, 4),
        intToCRSF(c.roll),
        c.roll,
        round(c.Phi, 4),
        c.throttleCRSF,
        c.throttle,
        c.a_x,
        c.a_y,
        c.a_z,
        c.w_y,
        c.Thrust
    ]

    with open("log.csv", "a", newline='') as csvfile:
        writer = csv.writer(csvfile)
        if not c.writtenHeaders:
            writer.writerow(headers)
            c.writtenHeaders = True
        writer.writerow(data)
