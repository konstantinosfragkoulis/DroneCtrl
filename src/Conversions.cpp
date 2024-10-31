#include <cmath>

#include "DroneCtrl.hpp"

using namespace DroneCtrl;

const float RPMtoThrustConst = (physics.PI * physics.RHO * (0.0762*0.0762) * 0.0635) / (3600 * 4 * 30);

// Calculate the thrust of a propeller given its RPM
float RPMtoThrust(int rpm) {
    return RPMtoThrustConst * std::pow(rpm, 2);
}

const float ThrustToRPMConst = sqrt((4 * 30 * 3600) / (physics.PI * physics.RHO * std::pow(0.0762, 2) * 0.0635));

// Calculate the RPM of a propeller given its thrust
int ThrustToRPM(float thrust) {
    return (int) (ThrustToRPMConst * std::sqrt(thrust));
}

int RPMtoThrottleCRSF(int rpm) {
    return (int) (0.02182329*rpm + 963.9417);
}

const float degToCRSFConst = 100/9;
// Convert degrees to CRSF values based on the current rate profile
// [-45, 45] -> [1000, 2000]
// This is used for the Roll and Pitch values
// For Yaw, you can use the same function, BUT INSTEAD OF DEGREES IT IS IN DEGREES PER SECOND
int degToCRSF(float deg) {
    if(deg < -45) deg = -45;
    if(deg > 45) deg = 45;

    return (int) (deg * degToCRSFConst + 1500);
}

const float CRSFToDegConst = 9/100;
// Convert CRSF values to degrees based on the current rate profile
// [1000, 2000] -> [-45, 45]
// This is used for the Roll and Pitch values
// For Yaw, you can use the same function, BUT INSTEAD OF DEGREES IT IS IN DEGREES PER SECOND
float CRSFToDeg(int crsf) {
    if(crsf < 1000) crsf = 1000;
    if(crsf > 2000) crsf = 2000;
    
    return (crsf - 1500) * CRSFToDegConst;
}

// Map CRFS values [1000, 2000] to int16 values [-32768, 32767]
int16_t CRSFToInt16(int val) {
    if(val < 1000) val = 1000;
    if(val > 2000) val = 2000;

    return static_cast<int16_t>((val - 1000) * 65.535 - 32768);
}