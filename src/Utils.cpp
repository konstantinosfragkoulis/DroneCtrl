#include "DroneCtrl.hpp"

#include <cstdio>

using namespace DroneCtrl;

float encodeState(State state, fState subState) {
    printf("State: %d, SubState: %d\n", static_cast<int>(state), static_cast<int>(subState));
    printf("State: %f\n", static_cast<float>(state) + static_cast<float>(subState) / 10);
    return static_cast<float>(state) + static_cast<float>(subState) / 10;
}

State decodeState(float state) {
    return static_cast<State>(static_cast<int>(state));
}

fState decodeSubState(float state) {
    float decimalPart = state - static_cast<int>(state);
    return static_cast<fState>((int) ((decimalPart * 10) + 0.01)); // Add 0.01 to avoid floating point errors
}

float remap(float value, float low1, float high1, float low2, float high2) {
    return low2 + (value - low1) * (high2 - low2) / (high1 - low1);
}

float clamp(float x, float a, float b) {
    return x < a ? a : (x > b ? b : x);
}