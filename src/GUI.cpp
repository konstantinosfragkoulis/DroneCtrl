#include "DroneCtrl.hpp"
#include "Utils.hpp"
#include <iostream>

using namespace DroneCtrl;

void sendDataToGUI() {
    /*
    "Roll", "Pitch", "Yaw", "Throttle",
    "Speed X", "Speed Y", "Speed Z", "Pos X",
    "Accel X", "Accel Y", "Accel Z", "Pos Y",
    "1", "2", "3", "Pos Z"
    */
    io.guiTx[0] = 0; // if(running) 0 else 1 but for this function to be called, the program must be running
    io.guiTx[1] = encodeState(sm.getState(), sm.getSubState());
    io.guiTx[2] = fd.roll;
    io.guiTx[3] = fd.pitch;
    io.guiTx[4] = fd.yaw;
    io.guiTx[5] = fd.throttle;
    io.guiTx[6] = fd.v[0];
    io.guiTx[7] = fd.v[1];
    io.guiTx[8] = fd.v[2];
    io.guiTx[9] = fd.pos[0];
    io.guiTx[10] = fd.a[0];
    io.guiTx[11] = fd.a[1];
    io.guiTx[12] = fd.a[2];
    io.guiTx[13] = fd.pos[1];
    io.guiTx[14] = fd.thrust;
    io.guiTx[15] = 0;
    io.guiTx[16] = 0;
    io.guiTx[17] = fd.pos[2];
    io.guiTx[18] = 0;
    io.guiTx[19] = 0;
    io.guiTx[20] = 0;
    io.guiTx[21] = 0;
    io.guiTx[22] = 0;
    io.guiTx[23] = 0;
    io.guiTx[24] = 0;
    io.guiTx[25] = 0;
    io.guiTx[26] = 0;
    io.guiTx[27] = 0;
    io.guiTx[28] = 0;
    io.guiTx[29] = 0;
    io.guiTx[30] = 0;
    io.guiTx[31] = 0;

    io.writeGUISharedMemory();
}

void handleGUI() {
    State curState = sm.getState();

    // Rx[0[]
    if(io.guiRx[0] != 0.0) {
        running = false;
    }

    // Rx[1]
    std::cout << "Rx[1]: " << io.guiRx[1] << std::endl;
    State targetState = decodeState(io.guiRx[1]);
    fState targetSubState = decodeSubState(io.guiRx[1]);

    if(targetState == State::Disarmed && curState == State::Grounded) {
        sm.setState(State::Disarmed);
    } else if(targetState == State::Grounded && curState == State::Disarmed) {
        sm.setState(State::Grounded);
    } else if(targetState == State::TakingOff && curState == State::Grounded) {
        sm.setState(State::TakingOff);
    } else if(targetState == State::Landing && curState == State::Flying) {
        sm.setState(State::Landing);
    }

    if(curState == State::Flying) {
        sm.setSubState(targetSubState);
    }

    // Rx[2], Rx[3]
    id.objCenter = {(int) io.guiRx[2], (int) io.guiRx[3]};

    // Rx[6], Rx[7], Rx[8]
    id.v = {io.guiRx[6], io.guiRx[7], io.guiRx[8]};
}