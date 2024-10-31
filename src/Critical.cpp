#include <cstdio>
#include <csignal>

#include <opencv2/opencv.hpp>

#include "DroneCtrl.hpp"
#include "Basic.hpp"

using namespace DroneCtrl;

void cleanup() {
    running = false;
    id.pos = fd.pos;
    id.v = {0, 0, 0};

    printf("Program stopped running\n");
    printf("Disarming...\n");
    Disarm();
    
    sm.setState(State::Disarmed); // Unnecessary

    printf("Closing Blackbox file...\n");
    bb.closeCSV();
    printf("Blackbox file closed\n");

    printf("Closing shared memory...\n");
    io.closeSharedMemory();
    printf("Shared memory closed\n");

    printf("Closing GUI shared memory...\n");
    io.closeGUISharedMemory();
    printf("GUI shared memory closed\n");

    printf("Releasing camera...\n");
    camera.releaseCamera();
    cv::destroyAllWindows();
    printf("Camera released\n");

    printf("Exiting...\n");
    exit(0);
}

void signalHandler(int signum) {
    printf("Interrupt signal (%d) received.\n", signum);
    cleanup();
}

void initSignalHandlers() {
    int signals[] = {SIGINT, SIGTERM, SIGHUP, SIGUSR1, SIGUSR2, SIGQUIT, SIGABRT, SIGSEGV, SIGFPE, SIGILL, SIGBUS};

    for (int signum : signals) {
        if (signal(signum, signalHandler) == SIG_ERR) {
            printf("Error: Unable to catch signal %d\n", signum);
        }
    }
}