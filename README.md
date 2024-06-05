# Dronectrl

DroneCtrl is a Python program for controlling and programming autonomous drones. It is designed to be easy to use and to work with a almost all FPV drones.

It is built to work with [ELRS Joystic Control](https://github.com/kaack/elrs-joystick-control), an awesome project that allows you to control your ELRS FPV drone from your computer using a joystick. In order for the two programs to communicate, I have modified ELRS Joystick Control, and you can get my fork [here](https://github.com/konfrag4/elrs-joystick-control).

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

Additionally, you need to have the ELRS Joystick Control program running on your computer. You can get it [here](https://github.com/konfrag4/elrs-joystick-control), where you can also find instructions on how to install it.

## Usage
To run the program, simply run the `main.py` file.
```
python3 main.py
```
Or you can run ```python3 main.py -v``` to log debug messages.

## Features
Currently, not much works. All you can do is:
- Automatic takeoff

## Future Features
The following features are planned to be implemented:
- Automatic landing
- Stabilized hover
- Object tracking
- Path planning
- Passing through hoops
