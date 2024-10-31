#ifndef UTILS_H
#define UTILS_H

#include "DroneCtrl.hpp"

using namespace DroneCtrl;

extern float encodeState(State state, fState subState);
extern State decodeState(float state);
extern fState decodeSubState(float state);
extern float remap(float value, float low1, float high1, float low2, float high2);
extern float clamp(float x, float a, float b);

#endif // UTILS_H