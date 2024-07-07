from config import *
from config import Config as c
from utils import clamp
from conversions import *
from basic import safeCall
import logging


@safeCall
def CalculateTakeoff(h: float, t: float):
    """Calculates the thrust needed for the drone to take off to a given
    height `h` in a given time `t`.""" 

    a1 = (3 * h)/(t ** 2)

    # This is not necessary for takeoff, but it is useful for debugging
    takeoffThrust1 = WEIGHT * (a1 + G)
    takeoffRPM1 = ThrustToRPM(takeoffThrust1/4) # Thrust per motor
    takeoffThrottle1 = CRSFtoInt(RPMtoThrottleCRSF(takeoffRPM1))
    u1 = (2 * a1 * t)/3

    a2  = -3 * u1/t # This, however, is necessary for takeoff
    takeoffThrust2 = WEIGHT * (a2 + G)
    takeoffRPM2 = ThrustToRPM(takeoffThrust2/4) # Thrust per motor
    takeoffThrottle2 = CRSFtoInt(RPMtoThrottleCRSF(takeoffRPM2))
    # End of unnecessary calculations

    c.takeoffAccel1 = clamp((a1/MAX_VERTICAL_ACCELERATION), -1, 1)
    c.takeoffAccel2 = clamp((a2/MAX_VERTICAL_ACCELERATION), -1, 1)

    logging.debug(f"\tTakeoff acceleration: {a1}")
    logging.debug(f"\tClamped takeoff acceleration: {c.takeoffAccel1}")
    logging.debug(f"\tTakeoff speed: {u1}", )
    logging.debug(f"\tTakeoff Thrust: {takeoffThrust1}")
    logging.debug(f"\tTakeoff RPM: {takeoffRPM1}")
    logging.debug(f"\tTakeoff Throttle CRSF: {intToCRSF(takeoffThrottle1)}")
    logging.debug(f"\tTakeoff Throttle: {takeoffThrottle1}")
    logging.debug(f"\tTarget thrust at takeoff: {takeoffThrust1}")
    logging.debug(f"\tPredicted thrust at takeoff: {Thrust(takeoffRPM1) * 4}")
    
    logging.debug("\n")

    logging.debug(f"\tTakeoff acceleration: {a2}")
    logging.debug(f"\tClamped takeoff acceleration: {c.takeoffAccel2}")
    logging.debug(f"\tTakeoff Thrust: {takeoffThrust2}")
    logging.debug(f"\tTakeoff RPM: {takeoffRPM2}")
    logging.debug(f"\tTakeoff Throttle CRSF: {intToCRSF(takeoffThrottle2)}")
    logging.debug(f"\tTakeoff Throttle: {takeoffThrottle2}")
    logging.debug(f"\tTarget thrust at takeoff: {takeoffThrust2}")
    logging.debug(f"\tPredicted thrust at takeoff: {Thrust(takeoffRPM2) * 4}")

    logging.debug("\n")

@safeCall
def Takeoff(takeoffStage: int):
    if takeoffStage == 1:
        logging.debug(f"\tSetting acceleration to {c.takeoffAccel1} for takeoff stage 1...")
        c.forward = 0
        c.vertical = c.takeoffAccel1
        c.angle = 0
    elif takeoffStage == 2:
        logging.debug(f"\tSetting acceleration to {c.takeoffAccel2} for takeoff stage 2...")
        c.forward = 0
        c.vertical = c.takeoffAccel2
        c.angle = 0
    else:
        logging.debug(f"\tInvalid takeoff stage {takeoffStage}. Must be 1 or 2")

# TODO: Actually calculate landing thrust
@safeCall
def CalculateLanding():
    """Calculates the thrust needed for the drone to land relatively slowly.""" 

    # landingThrottle1 = CRSFtoInt(RPMtoThrottle(ThrustToRPM(WEIGHT/4)))
    # landingThrottle2 = CRSFtoInt(RPMtoThrottle(ThrustToRPM(WEIGHT/4)))

    c.landingAccel1 = -0.2
    c.landingAccel2 = -0.1

    logging.debug(f"Landing Acceleration 1: {c.landingAccel1}")
    logging.debug(f"Landing Acceleration 2: {c.landingAccel2}")
    logging.debug("\n")

@safeCall
def Land(landingStage: int):
    if landingStage == 1:
        logging.debug(f"\tSetting acceleration to {c.landingAccel1} for landing stage 1...")
        c.forward = 0
        c.sideways = 0
        c.vertical = c.landingAccel1
        c.angle = 0
    elif landingStage == 2:
        logging.debug(f"\tSetting acceleration to {c.landingAccel2} for landing stage 2...")
        c.forward = 0
        c.sideways = 0
        c.vertical = c.landingAccel2
        c.angle = 0
    else:
        logging.debug(f"\tInvalid landing stage {landingStage}. Must be 1 or 2")
