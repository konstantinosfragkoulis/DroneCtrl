import sys
import cv2 as cv
import struct
import logging
from time import sleep
from config import *
from config import Config as c

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
