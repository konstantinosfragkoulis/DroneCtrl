import matplotlib.pyplot as plt
import pandas as pd
from conversions import Thrust, ThrustToRPM, RPMtoThrottleCRSF, CRSFtoInt, intToCRSF, intToDegPerSec, degPerSecToInt

def main():
    RPMvalues = range(1000, 11000, 1000)
    thrustValues = [Thrust(rpm) for rpm in RPMvalues]
    RPMcalc = [ThrustToRPM(thrust) for thrust in thrustValues]

    throttleValues = [RPMtoThrottleCRSF(rpm) for rpm in RPMvalues]

    CRSFvalues = range(1000, 2000, 100)
    intValues = [CRSFtoInt(crsf) for crsf in CRSFvalues]
    CRSFcalc = [intToCRSF(value) for value in intValues]

    DegPerSecValues = [intToDegPerSec(value) for value in intValues]
    intCalc = [degPerSecToInt(deg_sec) for deg_sec in DegPerSecValues]

    thrust = pd.DataFrame({"RPM": RPMvalues, "Thrust": thrustValues, "RPM Back": RPMcalc})
    throttle = pd.DataFrame({"RPM": RPMvalues, "Throttle CRSF": throttleValues})
    crsf = pd.DataFrame({"CRSF": CRSFvalues, "Int": intValues, "CRSF Back": CRSFcalc})
    degPersec = pd.DataFrame({"Int": intValues, "Deg/Sec": DegPerSecValues, "Int Back": intCalc})

    plt.figure(figsize=(10, 8))

    plt.subplot(2, 2, 1)
    plt.plot(RPMvalues, thrustValues, label="RPM to Thrust")
    plt.plot(RPMcalc, thrustValues, label="Thrust to RPM", dashes=(4, 5), color="red")
    plt.xlabel("RPM")
    plt.ylabel("Thrust")
    plt.title("RPM <-> Thrust Conversion")
    plt.grid(True)
    plt.legend()

    plt.subplot(2, 2, 2)
    plt.plot(RPMvalues, throttleValues, label="RPM to Throttle CRSF")
    plt.xlabel("RPM")
    plt.ylabel("Throttle CRSF")
    plt.title("RPM to Throttle CRSF")
    plt.grid(True)
    plt.legend()

    plt.subplot(2, 2, 3)
    plt.plot(CRSFvalues, intValues, label="CRSF to Int")
    plt.plot(CRSFcalc, intValues, label="Int to CRSF", dashes=(4, 5), color="red")
    plt.xlabel("CRSF")
    plt.ylabel("Int")
    plt.title("CRSF <-> Int Conversion")
    plt.grid(True)
    plt.legend()

    plt.subplot(2, 2, 4)
    plt.plot(intValues, DegPerSecValues, label="Int to Deg/Sec")
    plt.plot(intCalc, DegPerSecValues, label="Deg/Sec to Int", dashes=(4, 5), color="red")
    plt.xlabel("Int")
    plt.ylabel("Deg/Sec")
    plt.title("Int <-> Deg/Sec Conversion")
    plt.grid(True)
    plt.legend()

    print(thrust.describe())
    print(throttle.describe())
    print(crsf.describe())
    print(degPersec.describe())
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()