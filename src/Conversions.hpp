#ifndef CONVERSIONS_H
#define CONVERSIONS_H

extern float RPMtoThrust(int rpm);

extern int ThrustToRPM(float thrust);

extern int RPMtoThrottleCRSF(int rpm);

extern int degToCRSF(float deg);
extern float CRSFToDeg(int crsf);
extern int16_t CRSFToInt16(int val);

#endif // CONVERSIONS_H