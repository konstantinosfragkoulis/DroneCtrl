# DroneCtrl

DroneCtrl is a Python program for controlling and programming autonomous drones. It is designed to be easy to use and to work with almost all FPV drones.

It is built to work with [ELRS Joystic Control](https://github.com/kaack/elrs-joystick-control), an awesome project that allows you to control your ELRS FPV drone from your computer using a joystick. In order for the two programs to communicate, I have modified ELRS Joystick Control, and you can get my fork [here](https://github.com/konstantinosfragkoulis/elrs-joystick-control).

## Installation
To run the program, you need to have Python 3 installed on your computer.
Also, you need to install the following libraries:

- `cv2`
- `numpy`
- `nptyping`
- `posix_ipc`

Which can be installed using pip:
```
pip install opencv-python numpy nptyping posix_ipc
```

Additionally, you need to have the ELRS Joystick Control program running on your computer. You can get it [here](https://github.com/konstantinosfragkoulis/elrs-joystick-control), where you can also find instructions on how to install it.

## Usage
To run the program, simply run the `main.py` file.
```
python3 main.py
```
Or you can run ```python3 main.py -v``` to log debug messages.

IMPORTANT! For the program to work without any modifications, the drone needs to be in ANGLE mode. You can change that in Betaflight. Also, you need to make sure that the PID tuning is correct - that is make the throttle, yaw, pitch and roll curves linear. Without doing the above, the program will not work correctly and the drone will probably crash.

Additionally, you need to change some variables in the python script. Look for the section "USER EDITABLE VARIABLES". There, it is important to change the constant `MASS` to the mass of your drone in kilograms.

## Features
Currently, few things works. All you can do is:
- Automatic takeoff
- Automatic landing
- Controlling the drone with easy-to-use functions: Instead of manually setting the yaw, pitch, roll and throttle, you will just set the forward and vertical acceleration, and a helper function will do all of the conversions.

## Implemeted Features That are Currently Being Tested
Features which have been implemented and are currently being tested. Most likely there will be improvements.
- Object tracking
- Stabilized hover

## Features Currently Being Implemented
Features that are currently being implemented. If there are features for testing, they will be tested before implementing new ones.
- Passing through hoops

## Future Features
Features that are planned to be implemented:
- Path planning
