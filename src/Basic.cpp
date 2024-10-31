#include <thread>

#include "DroneCtrl.hpp"

using namespace DroneCtrl;

void Arm() {
    int values[16] = {0, 0, 0, -32768, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    io.passValues(values);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    values[6] = 1; // Set AUX1 to high (1)
    io.passValues(values);
}

void Disarm() {
    int values[16] = {0, 0, 0, -32768, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    io.passValues(values);
}