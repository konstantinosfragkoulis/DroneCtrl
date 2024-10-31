#include <csignal>

#include "DroneCtrl.hpp"

namespace DroneCtrl {
    double dt = 0.0;
    std::string VERSION = "0.0.1";
    bool running = true;

    bool SET_SPEED = true;

    float MAX_SPEED = 1.0;
    float MAX_ACCELERATION = 1.0;

    float takeOffHeight = 0.5;

    FlightData fd;
    MagicData md;
    InputData id;
    IOData io;
    Camera camera;
    Physics physics;
    Blackbox bb;
    StateManager sm;
}