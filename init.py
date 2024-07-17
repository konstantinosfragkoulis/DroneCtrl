from config import *
from config import Config as c
from basic import safeCall, handle_signal, cleanup, Arm
import logging
import posix_ipc
import mmap
import numpy as np
import struct
import cv2 as cv
import argparse
import sys
from utils import getFrame
from takeoff import CalculateTakeoff, CalculateLanding

@safeCall
def initSignalHandlers():
    """Initializes signal handlers for the program."""
    logging.debug("\tInitializing signal handlers...")
    for sig in signals_to_handle:
        signal.signal(sig, handle_signal)
    logging.debug("\tSignal handlers initialized")

@safeCall
def initSharedMemory():
    """Initializes shared memory for the program."""
    logging.debug("\tInitializing shared memory...") 
    c.memory = posix_ipc.SharedMemory("/myshm")
    c.mapFile = mmap.mmap(c.memory.fd, c.memory.size)
    c.values = struct.unpack('i'*16, c.mapFile.read(64))
    logging.debug("\tShared memory initialized")

@safeCall
def initCamera():
    """Initializes the camera for the program."""
    if c.virtual:
        logging.debug("\tInitializing virtual camera...")

        c.virtCamMemory = posix_ipc.SharedMemory("/myVirtCamMem", posix_ipc.O_CREAT, size=VIRTUAL_IMAGE_SIZE)
        c.virtCamMapFile = mmap.mmap(c.virtCamMemory.fd, VIRTUAL_IMAGE_SIZE)
        c.image = np.zeros((VIRTUAL_IMAGE_SIZE_X, VIRTUAL_IMAGE_SIZE_Y, VIRTUAL_IMAGE_SIZE_Z), dtype=np.uint8)

        logging.debug("\tVirtual camera initialized")
    else:
        logging.debug("\tInitializing camera...")
        
        c.cap = cv.VideoCapture("/dev/video0")
        
        logging.debug("\tCamera initialized")

@safeCall
def getFirstFrame():
    logging.debug("\tGetting first frame...")

    ret = getFrame()
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

@safeCall
def initArgumentParser():
    parser = argparse.ArgumentParser(
        description="A program that controls a drone and makes it autonomous."
    )
    parser.add_argument("-v", "--verbose", help="increase output verbosity", action="store_true")
    parser.add_argument("--virt", help="use virtual camera", action="store_true")
    parser.add_argument("--version", help="print the version of DroneCtrl", action="store_true")
    args = parser.parse_args()
    if args.verbose:
        logging.basicConfig(level=logging.DEBUG)
    if args.virt:
        c.virtual = True
    if args.version:
        print(f"DroneCtrl v{VERSION_NUMBER}")
        sys.exit(0)

    logging.getLogger('PIL').setLevel(logging.WARNING)
    logging.debug("\tVerbose mode")
