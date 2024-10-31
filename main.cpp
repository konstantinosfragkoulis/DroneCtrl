#include <cstdio>
#include <chrono>
#include <cstdlib>
#include <ctime>
#include <iostream>

#include <opencv2/opencv.hpp>

#include "src/DroneCtrl.hpp"
#include "src/test.hpp"
#include "src/Critical.hpp"
#include "src/Basic.hpp"
#include "src/Magic.hpp"
#include "src/ArgumentParser.hpp"
#include "src/GUI.hpp"
#include "src/Control.hpp"

using namespace DroneCtrl;

void Awake() {
    printf("DroneCtrl v%s Initializing...\n", VERSION.c_str());
    initSignalHandlers();
}

void Start() {
    bb.initializeBlackbox();
    printf("Initialized Blackbox\n");

    io.initSharedMemory();
    printf("Initialized Shared memory\n");

    io.initGUISharedMemory();
    printf("Initialized GUI Shared memory\n");

    Disarm();
    sm.setState(State::Disarmed);
    printf("Disarmed Drone \n");

    // srand(static_cast<unsigned int>(time(0)));

    // if(!camera.initializeCamera()) {
    //     printf("Failed to initialize camera\n");
    //     cleanup();
    // }
}

void Update() {
    // someFunction();
    // readSharedMemory(); There is no need to read the Shared memory. DroneCtrl only writes to it.
    io.readGUISharedMemory();
    handleGUI();

    cv::Mat image;

    // camera.getFrame(image);
    // if(!image.empty()) {
    //     cv::imshow("Camera", image);
    //     cv::waitKey(1);
    // }

    State curState = sm.getState();
    if (curState == State::Disarmed) {
        printf("Drone is disarmed\n");
        Disarm();
        io.values[6] = 0; // Disarm
    } else if (curState == State::Grounded) {
        for(int i = 0; i < 16; i++) {
            io.values[i] = 0;
        }
        io.values[3] = -32768; // Throttle
        io.values[6] = 32767; // Arm
        io.passValues(io.values);
    } else if (curState == State::TakingOff) {
        TakeOff();
        // camera.saveFrameToPNG("");
    } else if (curState == State::Flying) {
        
        if (sm.getSubState() == fState::Hovering) {
            Hover();
        } else if (sm.getSubState() == fState::StabilizedHover) {
            StabilizedHover();
        } else if (sm.getSubState() == fState::FollowingObject) {
            FollowObject();
        } else if (sm.getSubState() == fState::FlyingForward) {
            FlyForward(0.25);
        } // if manual control, do nothing

    } else if (curState == State::Landing) {
        Land();
    }

    printf("dt: %f\n", dt);



    doMagic();

    // printf("Desired Speed: %f, %f, %f\n", id.v[0], id.v[1], id.v[2]);
    // printf("DV: %f, %f, %f\n", md.dv[0], md.dv[1], md.dv[2]);
    // printf("Position: %f, %f, %f\n", fd.pos[0], fd.pos[1], fd.pos[2]);
    // printf("Velocity:\t\t\t\t %f, %f, %f\n", fd.v[0], fd.v[1], fd.v[2]);
    // printf("Yaw: %f\n", fd.w);
    // printf("Acceleration: %f, %f, %f\n", fd.a[0], fd.a[1], fd.a[2]);
    // printf("Thrust: %f\n", md.thrust);
    // printf("Theta: %f\n", md.theta);
    // printf("Phi: %f\n", md.phi);
    // printf("F: %f, %f, %f\n", md.F[0], md.F[1], md.F[2]);

    sendDataToGUI();
    // for(int i = 0; i < 32; i++) {
    //     printf("GUI RX: %f\n", io.guiRx[i]);
    // }

    printf("Object Center: %d, %d\n", id.objCenter[0], id.objCenter[1]);

    // TODO: C++ is very fast therefore the CSV file becomes huge very quickly.
    // More than 1.7 MiB in 1 second. This is not a good idea.
    bb.writeFlightDataToCSV();
    // printf("Roll: %d, Pitch: %d, Yaw: %d, Throttle: %d\n", io.values[0], io.values[1], io.values[2], io.values[3]);
}

void UpdateHelp() {
    auto initialTime = std::chrono::high_resolution_clock::now();
    while(running) {
        Update();

        auto currentTime = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsedTime = currentTime - initialTime;
        dt = elapsedTime.count();
        initialTime = currentTime;
    }

    cleanup();
}

int main(int argc, char *argv[]) {
    parseArguments(argc, argv);

    Awake();
    Start();
    UpdateHelp();

    return 0;
}
