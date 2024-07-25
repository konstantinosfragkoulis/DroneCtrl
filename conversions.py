import math
from config import PI, RHO
from basic import safeCall, cleanup

#################################################################
###############  CONVERSION FUNCTIONS - START  ##################
#################################################################
const = (PI * RHO * (0.0762**2) * 0.0635) / (3600 * 4 * 50) # * (RPM**2)
@safeCall
def Thrust(rpm: int):
    """Calculates the thrust produced by a motor spinning at a given RPM."""
    # Simplified thrust formula: T = (pi * rho * D^2 * n^2 * P) / 4
    global const
    return const * (rpm ** 2)

const2 = math.sqrt((4 * 50 * 3600) / (PI * RHO * (0.0762 ** 2) * 0.0635)) # * sqrt(RPM)
@safeCall
def ThrustToRPM(thrust: float):
    """Calculates the RPM of a motor needed to produce the given thrust by it."""
    global const2
    return int((math.sqrt(thrust)) * const2)

const3 = 1/(10196720000**1.011358)
@safeCall
def RPMtoThrottleCRSF(rpm: int):
    """Converts the RPM of a motor to a CRSF throttle value [1000, 2000]."""
    global const3
    return int(270782500 + (-270781529.4186)/(1 + rpm**1.011358 * const3))

const8 = 65535 / 1000
const9 = 32768
@safeCall
def CRSFtoInt(value: int):
    """Converts from CRSF [1000, 2000] to int16 [-32768, 32767]."""
    return int(((value - 1000) * const8) - const9)

const4 = 1000 / 65535
const5 = 1000 + (32768 * const4)
@safeCall             
def intToCRSF(value: int):
    """Converts from int16 [-32768, 32767] to CRSF [1000, 2000]."""
    return int((value * const4) + const5)

const6 = 360/(32767+32768)
@safeCall
def intToDegPerSec(value: int):
    """Converts Pitch, Roll, and Yaw values [-32768, 32767] to degrees per second."""
    if value > 32767 or value < -32768:
        print(f"\tInvalid value {value}. Must be between -32768 and 32767.")
        cleanup()
        return
    return const6*value

const7 = (32767+32768)/360
@safeCall
def degPerSecToInt(value: float):
    """Converts degrees per second to int16 [-32768, 32767]."""
    if value > 180 or value < -180:
        print(f"\tInvalid value {value}. Must be between -180 and 180.")
        cleanup()
        return
    return int(const7 * value)
