import math
from config import PI, RHO
from basic import safeCall

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

@safeCall
def CRSFtoInt(value: int):
    """Converts from CRSF [1000, 2000] to int16 [-32768, 32767]."""
    return int(52.88747*value - 79333.9)
  
@safeCall             
def intToCRSF(value: int):
    """Converts from int16 [-32768, 32767] to CRSF [1000, 2000]."""
    return int(0.01890788*value + 1500.051)

@safeCall
def intToDegPerSec(value: int):
    """Converts Pitch, Roll, and Yaw values [-32768, 32767] to degrees per second."""
    return 0.02534762*value + 0.02909812

const4 = 0.02909812/0.02534762
@safeCall
def degPerSecToInt(value: float):
    """Converts degrees per second to int16 [-32768, 32767]."""
    return int(value/0.02534762 - const4)
