#include <cmath>

#include "DroneCtrl.hpp"
#include "Conversions.hpp"

using namespace DroneCtrl;

float calculateAcceleration(float dv) {
    // Calculate the acceleration needed to reach the desired speed within 1 second. If it exceeds the maximum acceleration, return the maximum acceleration
    // Since acceleration is constant: a = dv / t
    // But t = 1s, therefore a = dv
    if(dv > MAX_ACCELERATION) {
        return MAX_ACCELERATION;
    } else if(dv < -MAX_ACCELERATION) {
        return -MAX_ACCELERATION;
    } else {
        return dv;
    }
}

void doMagic() {
    if(!running || sm.getState() == State::Grounded || sm.getState() == State::Disarmed) {
        // These are int16 values, so 0 is the middle position
        io.values[0] = 0;
        io.values[1] = 0;
        io.values[2] = 0;
        io.values[3] = -32768;

        io.passValues(io.values);
        
        fd.v = {0, 0, 0};
        fd.a = {0, 0, 0};
        fd.w = 0;
        fd.thrust = 0;

        fd.theta = 0;
        fd.phi = 0;

        fd.roll = 0;
        fd.pitch = 0;
        fd.yaw = 0;
        fd.throttle = 0;


        return;
    }
    if(SET_SPEED) {
        md.dv = {
            id.v[0] - fd.v[0],
            id.v[1] - fd.v[1],
            id.v[2] - fd.v[2]
        };

        md.a = {
            calculateAcceleration(md.dv[0]),
            calculateAcceleration(md.dv[1]) + physics.G,
            calculateAcceleration(md.dv[2])
        };
        fd.a = md.a;

        md.F = {
            physics.MASS * md.a[0],
            physics.MASS * md.a[1],
            physics.MASS * md.a[2]
        };

        md.theta = atan2(md.F[2], md.F[1]) * physics.RAD2DEG;
        fd.pitch = (md.theta);
        md.phi = atan2(md.F[0], md.F[1]) * physics.RAD2DEG;
        fd.roll = (md.phi);

        fd.v = {
            fd.v[0] + md.a[0] * (float) dt,
            fd.v[1] + (md.a[1] - physics.G) * (float) dt,
            fd.v[2] + md.a[2] * (float) dt
        };

        fd.pos = {
            fd.pos[0] + fd.v[0] * static_cast<float>(dt),
            fd.pos[1] + fd.v[1] * static_cast<float>(dt),
            fd.pos[2] + fd.v[2] * static_cast<float>(dt)
        };

        md.thrust = sqrt(pow(md.F[0], 2) + pow(md.F[1], 2) + pow(md.F[2], 2));
        fd.thrust = md.thrust;

        fd.yaw = 0;
        fd.throttle = 1000; // TODO: Implement a function to calculate the throttle based on target thrust and use it here
        fd.throttle = RPMtoThrottleCRSF(ThrustToRPM(md.thrust));

        io.values[0] = CRSFToInt16(degToCRSF(fd.roll));
        io.values[1] = CRSFToInt16(degToCRSF(fd.pitch));
        io.values[2] = CRSFToInt16(degToCRSF(fd.yaw));
        io.values[3] = CRSFToInt16(fd.throttle);
        io.values[6] = 32767; // Arm

        io.passValues(io.values);
    } else {
        // We don't move sideways, only up/down and forward/backward
        // Instead, we turn about the y axis (yaw).
    }
}