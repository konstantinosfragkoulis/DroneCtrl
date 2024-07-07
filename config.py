import signal
from enum import IntEnum
from dataclasses import dataclass

#################################################################
##################  PHYSICS CONSTANTS - START  ##################
#################################################################
PI = 3.14159
G = 9.81 # m/s^2
RHO = 1.225 # kg/m^3
DEG_TO_RAD = 180/PI
#################################################################
###################  PHYSICS CONSTANTS - END  ###################
#################################################################





#################################################################
###############  USER EDITABLE VARIABLES - START  ###############
#################################################################
TAKEOFF_TIME = 0.6 # The time it takes for the drone to take off
TAKEOFF_HEIGHT = 0.1 # The height the drone will take off to in meters
MASS = 0.31 # The mass of the drone in kg
LANDING_TIME = 3 # The drone will be descending for this time

MAX_FORWARD_ACCELERATION = 2 # m/s^2
MAX_SIDEWAYS_ACCELERATION = 2 # m/s^2
MAX_VERTICAL_ACCELERATION = 2 # m/s^2
MAX_ANGULAR_ACCELERATION = 90 # degrees/s^2

VERTICAL_ACCELERATION_OFFSET = 0.5 # The drone hovers at this acceleration

STABILIZED_HOVER_STEP_ACCELERATION = 0.5 # m/s^2
STABILIZED_HOVER_STEP_DURATION = 0.5 # s
#################################################################
################  USER EDITABLE VARIABLES - END  ################
#################################################################

STABILIZED_HOVER_STEP_DURATIOND2 = STABILIZED_HOVER_STEP_DURATION / 2
STABILIZED_HOVER_STEP_ACCELERATIOND2 = STABILIZED_HOVER_STEP_ACCELERATION / 2
TAKEOFF_TIME1 = 2 * TAKEOFF_TIME / 3
TAKEOFF_TIME2 = TAKEOFF_TIME / 3
WEIGHT = MASS * G
LANDING_TIME1 = LANDING_TIME / 3

CAM_WIDTH = 640
CAM_HEIGHT = 480
CAM_WIDTHD2 = CAM_WIDTH / 2
CAM_HEIGHTD2 = CAM_HEIGHT / 2

PURPLE = ((130, 150, 150), (140, 255, 255))
ORANGE = ((10, 150, 150), (20, 255, 255))
RED = ((170, 125, 125), (10, 255, 255))
GREEN = ((55, 150, 150), (65, 255, 255))
BLUE = ((100, 110, 110), (115, 255, 255))

signals_to_handle = [
    signal.SIGINT,
    signal.SIGTERM,
    signal.SIGHUP,
    signal.SIGUSR1,
    signal.SIGUSR2,
    signal.SIGQUIT,
    signal.SIGABRT
]

class State(IntEnum):
    Disarmed = 0
    Grounded = 1
    TakingOff = 2
    Flying = 3
    Landing = 4

class FlyingState(IntEnum):
    Hovering = 0
    FollowingObject = 1
    FlyingForward = 2
    StabilizedHover = 3

@dataclass
class StablizedHoverData:
    centerY: int
    centerX: int
    dCenterY: int
    dCenterX: int
    adjustedHoverData:bool
    d: float
    h: float
    accelY: float
    accelX: float
    stabilizationDuration: float


class Config:
    state = State.Disarmed

    flyingState = FlyingState.Hovering

    running = True

    memory = None
    mapFile = None
    values = None
    cap = None

    forward = 0 # Move front or back
    sideways = 0 # Move left or right
    vertical = -10 # Move up or down
    angle = 0 # Turn left or right

    height = 0 # The calculated height of the drone

    image = None # The current image from the camera

    dt = 0 # The time difference between to Update calls

    #################################################################
    ############  TAKEOFF AND LANDING VARIABLES - START  ############
    #################################################################
    takeoffCnt = 0
    takeoffAccel1: float = 0
    takeoffAccel2: float = 0

    landingCnt = 0
    landingAccel1: float = 0
    landingAccel2: float = 0
    #################################################################
    #############  TAKEOFF AND LANDING VARIABLES - END  #############
    #################################################################

    a_x = 0
    a_y = 0
    a_z = 0
    w_y = 0

    Thrust = 0
    ThrustXZ = 0
    ThrustYZ = 0
    rpm = 0
    throttle = 0
    throttleCRSF = 0
    Theta = 0
    Phi = 0
    pitch = 0
    yaw = 0
    roll = 0

    startTime = None

    virtualCam = False
    virtCamMemory = None
    virtCamMapFile = None

    debugInfo = None

    stabilizedHoverTime = 0
    hd = StablizedHoverData(0, 0, 0, 0, False, 0, 0, 0, 0, 0)
