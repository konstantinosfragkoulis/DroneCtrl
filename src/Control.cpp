#include <cstdio>

#include "DroneCtrl.hpp"
#include "Critical.hpp"
#include "Utils.hpp"

using namespace DroneCtrl;

void TakeOff() {
    if(sm.getState() != State::TakingOff) {
        fprintf(stderr, "Drone is not in the TakingOff state\n");
        cleanup();
    }

    if(fd.pos[1] < takeOffHeight) {
        id.v = {0, MAX_SPEED/2, 0};
        id.yaw = 0;
    } else {
        id.v = {0, 0, 0};
        id.yaw = 0;
        sm.setState(State::Flying);
    }

}

void Land() {
    if(sm.getState() != State::Landing) {
        fprintf(stderr, "Drone is not in the Landing state\n");
        cleanup();
    }

    if(fd.pos[1] > 0.1) {
        id.v = {0, -MAX_SPEED, 0};
        id.yaw = 0;
    } else {
        id.v = {0, 0, 0};
        id.yaw = 0;
        sm.setState(State::Grounded);
    }
}

void Hover() {
    if(sm.getState() != State::Flying || sm.getSubState() != fState::Hovering) {
        fprintf(stderr, "Drone is not in Flying state or Hovering fState\n");
        cleanup();
    }

    id.v = {0, 0, 0};
}

void StabilizedHover() {
    if(sm.getState() != State::Flying || sm.getSubState() != fState::StabilizedHover) {
        fprintf(stderr, "Drone is not in Flying state or StabilizedHover fState\n");
        cleanup();
    }

    int dx = id.objCenter[0] - camera.widthD2;
    int dy = id.objCenter[1] - camera.heightD2;

    id.v = {
        remap(dx, -camera.widthD2, camera.widthD2, -MAX_SPEED, MAX_SPEED),
        remap(dy, -camera.heightD2, camera.heightD2, -MAX_SPEED, MAX_SPEED),
        0
    };
}

// This needs improvement
void FollowObject() {
    if(sm.getState() != State::Flying || sm.getSubState() != fState::FollowingObject) {
        fprintf(stderr, "Drone is not in Flying state or FollowingObject fState\n");
        cleanup();
    }

    int dx = id.objCenter[0] - camera.widthD2;
    int dy = id.objCenter[1] - camera.heightD2;

    id.v = {
        remap(dx, -camera.widthD2, camera.widthD2, -MAX_SPEED, MAX_SPEED),
        remap(dy, -camera.heightD2, camera.heightD2, -MAX_SPEED, MAX_SPEED),
        MAX_SPEED/2
    };
}

void FlyForward(float speed) {
    if(sm.getState() != State::Flying || sm.getSubState() != fState::FlyingForward) {
        fprintf(stderr, "Drone is not in Flying state or FlyingForward fState\n");
        cleanup();
    }

    speed = clamp(speed, -MAX_SPEED, MAX_SPEED);
    id.v = {speed, 0, 0};
}