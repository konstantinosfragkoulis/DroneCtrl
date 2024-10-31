import tkinter as tk
from tkinter import ttk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import numpy as np
import random
import time

import posix_ipc
import mmap
import struct

DISARMED = 0
ARMED = 1
TAKEOFF = 2
FLYING = 3
LANDING = 4

HOVERING = 0
FOLLOWING_OBJECT = 1
FLYING_FORWARD = 2
STABILIZED_HOVER = 3
MANUAL_CONTROL = 4

# Global variables
Root = None
PlotLabels = [
    "Roll (Deg)", "Pitch (Deg)", "Yaw (Deg)", "Throttle (%)",
    "Speed X (m/s)", "Speed Y (m/s)", "Speed Z (m/s)", "Pos X (m)",
    "Accel X (m/s^2)", "Accel Y (m/s^2)", "Accel Z (m/s^2)", "Pos Y (m)",
    "Thrust (N)", "2", "3", "Pos Z (m)"
]
Axes = []
CanvasPlots = None
PlotData = [np.zeros(100) for _ in range(16)]
SpeedXEntry = None
SpeedYEntry = None
SpeedZEntry = None
PosEntry = None

WINDOW_NAME = "DroneCtrl"
WINDOW_GEOMETRY = "1280x720"

memory = None
mapFile = None
guiTx = [0.0] * 32
guiRx = [0.0] * 32


#########################################
### DroneCtrl Data from Shared Memory ###
#########################################
running = True
state = 0.0
roll = 0.0
pitch = 0.0
yaw = 0.0
throttle = 0.0
v = [0.0, 0.0, 0.0]
pos = [0.0, 0.0, 0.0]
a = [0.0, 0.0, 0.0]
thrust = 0.0
#########################################
### DroneCtrl Data from Shared Memory ###
#########################################

#########################################
####### Data to send to DroneCtrl #######
#########################################
running_tx = True
state_tx = 0.0
v_tx = [0.0, 0.0, 0.0]
#########################################
####### Data to send to DroneCtrl #######
#########################################

def passValues(*inputs):
    global memory, mapFile, guiTx
    guiTx = list(map(float, inputs)) + [0.0] * (32 - len(inputs))
    mapFile.seek(0)
    mapFile.write(struct.pack('f'*32, *guiTx))

def copyValues():
    global memory, mapFile, guiTx
    global running_tx, state_tx, v_tx
    
    if running_tx:
        guiTx[0] = 0.0
    else:
        guiTx[0] = 1.0
    
    guiTx[1] = state_tx
    guiTx[2] = 0
    guiTx[3] = 0
    guiTx[4] = 0
    guiTx[5] = 0    
    guiTx[6] = v_tx[0]
    guiTx[7] = v_tx[1]
    guiTx[8] = v_tx[2]
    guiTx[9] = 0
    guiTx[10] = 0
    guiTx[11] = 0
    guiTx[12] = 0
    guiTx[13] = 0
    guiTx[14] = 0
    guiTx[15] = 0
    guiTx[16] = 0
    guiTx[17] = 0
    guiTx[18] = 0
    guiTx[19] = 0
    guiTx[20] = 0
    guiTx[21] = 0
    guiTx[22] = 0
    guiTx[23] = 0
    guiTx[24] = 0
    guiTx[25] = 0
    guiTx[26] = 0
    guiTx[27] = 0
    guiTx[28] = 0
    guiTx[29] = 0
    guiTx[30] = 0
    guiTx[31] = 0

    mapFile.seek(0)
    mapFile.write(struct.pack('f'*32, *guiTx))

def readGuiTx():
    global running_tx, state_tx, v_tx
    global memory, mapFile, guiTx
    mapFile.seek(0)
    data = mapFile.read(32 * 4)
    if len(data) != 32 * 4:
        raise ValueError(f"Expected 128 bytes, but got {len(data)} bytes")
    tmpArr = struct.unpack('f'*32, data)
    print("PosX: ", tmpArr[2], "PosY: ", tmpArr[3])

def readGuiRx():
    global running, state, roll, pitch, yaw, throttle, v, pos, a, thrust
    global memory, mapFile, guiRx
    mapFile.seek(32 * 4)
    data = mapFile.read(128)
    if len(data) != 128:
        raise ValueError(f"Expected 128 bytes, but got {len(data)} bytes")
    guiRx = struct.unpack('f'*32, data)

    if guiRx[0] != 0.0:
        running = False
    else:
        running = True
    
    state = guiRx[1]
    roll = guiRx[2]
    pitch = guiRx[3]
    yaw = guiRx[4]
    throttle = guiRx[5]
    v = [guiRx[6], guiRx[7], guiRx[8]]
    pos = [guiRx[9], guiRx[13], guiRx[17]]
    a = [guiRx[10], guiRx[11], guiRx[12]]
    thrust = guiRx[14]

def initSharedMemory():
    global memory, mapFile, guiTx, guiRx

    memory = posix_ipc.SharedMemory("/guiSharedMem")
    mapFile = mmap.mmap(memory.fd, memory.size)
    
    copyValues()
    readGuiRx()

    for i, value in enumerate(guiRx):
        print(f"guiRx[{i}]: {value}")

initSharedMemory()

def encodeState(state, substate) -> float:
    return float(state + substate/10)

def decodeState(state: float) -> tuple:
    return int(state), int((state - int(state)) * 10)

def Disarm():
    global state, state_tx
    print("Disarm")
    state_tx = encodeState(DISARMED, HOVERING)
    copyValues()

def Arm():
    global state, state_tx
    if decodeState(state)[0] != DISARMED:
        print("Drone is not disarmed. Cannot arm.")
        return
    print("Arm")
    state_tx = encodeState(ARMED, HOVERING)
    copyValues()

def takeOff():
    global state, state_tx
    if decodeState(state)[0] != ARMED:
        print("Drone must be armed to take off.")
        return
    print("Take Off")
    state_tx = encodeState(TAKEOFF, HOVERING)
    copyValues()

def land():
    global state, state_tx
    if decodeState(state)[0] != FLYING:
        print("Drone must be flying to land.")
        return
    print("Land")
    state_tx = encodeState(LANDING, HOVERING)
    copyValues()

def followObject():
    global state, state_tx
    if decodeState(state)[0] != FLYING:
        print("Drone must be flying to follow object.")
        return
    print("Follow Object")
    state_tx = encodeState(FLYING, FOLLOWING_OBJECT)
    print(f"state_tx: {state_tx}")
    print("\n\n\n\n\n\n")
    copyValues()

def stabilizedHover():
    global state, state_tx
    if decodeState(state)[0] != FLYING:
        print("Drone must be flying to stabilized hover.")
        return
    print("Stabilized Hover")
    state_tx = encodeState(FLYING, STABILIZED_HOVER)
    copyValues()

def flyForward():
    global state, state_tx
    if decodeState(state)[0] != FLYING:
        print("Drone must be flying to fly forward.")
        return
    print("Fly Forward")
    state_tx = encodeState(FLYING, FLYING_FORWARD)
    copyValues()

def hover():
    global state, state_tx
    if decodeState(state)[0] != FLYING:
        print("Drone must be flying to hover.")
        return
    print("Hover")
    state_tx = encodeState(FLYING, HOVERING)
    copyValues()

def manualControl():
    global state, state_tx
    if decodeState(state)[0] != FLYING:
        print("Drone must be flying to manual control.")
        return
    print("Manual Control")
    state_tx = encodeState(FOLLOWING_OBJECT, MANUAL_CONTROL)
    copyValues()

def emergencyStop():
    global running_tx
    print("Emergency Stop")
    running_tx = False
    copyValues()

def updateSpeed():
    global guiTx
    global SpeedXEntry, SpeedYEntry, SpeedZEntry
    global v_tx
    if decodeState(state)[0] != FLYING:
        print("Drone must be flying to update speed.")
        return
    try:
        v_tx[0] = float(SpeedXEntry.get())
        v_tx[1] = float(SpeedYEntry.get())
        v_tx[2] = float(SpeedZEntry.get())
        copyValues()
        print(f"Updated Speed: {v_tx}")
    except ValueError:
        print("Invalid input. Please enter valid float values.")

def onKeyPress(event):
    pass
    emergencyStop()

def initGUI():
    global memory, mapFile, guiTx, guiRx
    global Root, PlotLabels, Axes, CanvasPlots, PlotData
    global SpeedXEntry, SpeedYEntry, SpeedZEntry, PosEntry
    
    global running_tx, state_tx
    
    running_tx = True
    state_tx = encodeState(DISARMED, HOVERING)
    copyValues()

    Root = tk.Tk()
    Root.title(WINDOW_NAME)
    Root.geometry(WINDOW_GEOMETRY)

    Root.grid_columnconfigure(1, weight=1)
    Root.grid_rowconfigure(0, weight=1)

    FrameControls = ttk.Frame(Root)
    FrameControls.grid(row=0, column=0, padx=10, pady=10, sticky="nw")

    FramePlots = ttk.Frame(Root)
    FramePlots.grid(row=0, column=1, padx=10, pady=10, sticky="nsew")
    FramePlots.grid_columnconfigure(0, weight=1)
    FramePlots.grid_rowconfigure(0, weight=1)

    ttk.Label(FrameControls, text="Target Speed X:").grid(row=0, column=0, padx=5, pady=5)
    SpeedXEntry = ttk.Entry(FrameControls)
    SpeedXEntry.grid(row=0, column=1, pady=(0, 5))

    ttk.Label(FrameControls, text="Target Speed Y:").grid(row=1, column=0, padx=5, pady=5)
    SpeedYEntry = ttk.Entry(FrameControls)
    SpeedYEntry.grid(row=1, column=1, pady=(0, 5))

    ttk.Label(FrameControls, text="Target Speed Z:").grid(row=2, column=0, padx=5, pady=5)
    SpeedZEntry = ttk.Entry(FrameControls)
    SpeedZEntry.grid(row=2, column=1, pady=(0, 5))

    ttk.Label(FrameControls, text="Target Position:").grid(row=3, column=0, padx=5, pady=5)
    PosEntry = ttk.Entry(FrameControls)
    PosEntry.grid(row=3, column=1, pady=(0, 5))

    ttk.Button(FrameControls, text="Update Speeds", command=updateSpeed).grid(row=4, column=0, columnspan=2, pady=(5, 10))
    ttk.Button(FrameControls, text="Disarm", command=Disarm).grid(row=5, column=0, pady=(5, 5))
    ttk.Button(FrameControls, text="Arm", command=Arm).grid(row=5, column=1, pady=(5, 5))
    ttk.Button(FrameControls, text="Take Off", command=takeOff).grid(row=6, column=0, pady=(5, 5))
    ttk.Button(FrameControls, text="Land", command=land).grid(row=6, column=1, pady=(5, 5))
    ttk.Button(FrameControls, text="Follow Object", command=followObject).grid(row=7, column=0, pady=(5, 5))
    ttk.Button(FrameControls, text="Stabilized Hover", command=stabilizedHover).grid(row=7, column=1, pady=(5, 5))
    ttk.Button(FrameControls, text="Fly Forward", command=flyForward).grid(row=8, column=0, pady=(5, 5))
    ttk.Button(FrameControls, text="Hover", command=hover).grid(row=8, column=1, pady=(5, 5))
    ttk.Button(FrameControls, text="Manual Control", command=manualControl).grid(row=9, column=0, columnspan=2, pady=(5, 10))
    ttk.Button(FrameControls, text="Emergency Stop", command=emergencyStop).grid(row=10, column=0, columnspan=2, pady=(5, 10))

    FigPlots = Figure(figsize=(12, 12))
    
    for i in range(4):
        for j in range(4):
            Ax = FigPlots.add_subplot(4, 4, i * 4 + j + 1)
            Ax.set_xlim(0, 100)
            Ax.set_ylim(-100, 100)
            Ax.set_title(PlotLabels[i * 4 + j])
            Axes.append(Ax)

    FigPlots.subplots_adjust(left=0.05, right=0.95, top=0.95, bottom=0.05, wspace=0.3, hspace=0.3)

    CanvasPlots = FigureCanvasTkAgg(FigPlots, master=FramePlots)
    CanvasPlots.get_tk_widget().grid(row=0, column=0, sticky="nsew")

    Root.bind("<KeyPress>", onKeyPress)

    updatePlots()

    Root.mainloop()

def updatePlots():
    """Update the plots with the current data from guiRx"""
    global PlotData, CanvasPlots, Axes, PlotLabels, guiRx

    readGuiRx()

    for i, value in enumerate(guiRx):
        print(f"guiRx[{i}]: {value}")
    readGuiTx()

    """
    PlotLabels = [
        "Roll (Deg)", "Pitch (Deg)", "Yaw (Deg)", "Throttle (%)",
        "Speed X (m/s)", "Speed Y (m/s)", "Speed Z (m/s)", "Pos X (m)",
        "Accel X (m/s^2)", "Accel Y (m/s^2)", "Accel Z (m/s^2)", "Pos Y (m)",
        "Thrust (N)", "2", "3", "Pos Z (m)"
    ]
    """

    for i, Ax in enumerate(Axes):
        PlotData[i] = np.roll(PlotData[i], -1)
        PlotData[i][-1] = guiRx[i+2]
        Ax.clear()
        Ax.plot(range(len(PlotData[i])), PlotData[i])
        Ax.set_title(PlotLabels[i])
        Ax.set_xlim(0, 100)
        if(i < 3): # Roll, Pitch, Yaw
            Ax.set_ylim(-45, 45)
        elif i == 3: # Throttle
            Ax.set_ylim(1000, 2000)
        elif i == 7 or i == 11 or i == 15: # Position
            Ax.set_ylim(-20, 20)
        elif i == 8 or i == 10: # Acceleration
            Ax.set_ylim(-2, 2)
        elif i == 9: # Acceleration Y (The graph includes g)
            Ax.set_ylim(0, 12)
        elif i == 12: # Thrust
            Ax.set_ylim(0, 5)
        else: # Speed
            Ax.set_ylim(-10, 10)

    CanvasPlots.draw()

    Root.after(100, updatePlots)

initGUI()