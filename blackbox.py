import pandas as pd
import matplotlib.pyplot as plt

data = pd.read_csv("log.csv")

plt.figure(figsize=(15, 15))

plt.subplot(3, 3, 1)
plt.plot(data["Timestamp"], data["Acc X"], label="Acc X")
plt.title("Acceleration X")
plt.ylabel("m/s^2")
plt.legend()
plt.xticks([])

plt.subplot(3, 3, 2)
plt.plot(data["Timestamp"], data["Acc Y"], label="Acc Y")
plt.title("Acceleration Y")
plt.ylabel("m/s^2")
plt.legend()
plt.xticks([])

plt.subplot(3, 3, 3)
plt.plot(data["Timestamp"], data["Acc Z"], label="Acc Z")
plt.title("Acceleration Z")
plt.ylabel("m/s^2")
plt.legend()
plt.xticks([])

plt.subplot(3, 3, 4)
plt.plot(data["Timestamp"], data["Pitch Angle"], label="Pitch Angle")
plt.title("Pitch Angle")
plt.ylabel("Degrees")
plt.legend()
plt.xticks([])

plt.subplot(3, 3, 5)
plt.plot(data["Timestamp"], data["Roll Angle"], label="Roll Angle")
plt.title("Roll Angle")
plt.ylabel("Degrees")
plt.legend()
plt.xticks([])

plt.subplot(3, 3, 6)
plt.plot(data["Timestamp"], data["Yaw Angle"], label="Yaw Angle")
plt.title("Yaw Angle")
plt.ylabel("Degrees")
plt.legend()
plt.xticks([])

plt.subplot(3, 3, 7)
plt.plot(data["Timestamp"], data["Throttle CRSF"], label="Throttle CRSF")
plt.title("Throttle")
plt.ylabel("CRSF")
plt.legend()
plt.xticks([])

plt.subplot(3, 3, 8)
plt.plot(data["Timestamp"], data["W Y"], label="W Y")
plt.title("Yaw int16")
plt.ylabel("int16")
plt.legend()
plt.xticks([])

plt.subplot(3, 3, 9)
plt.plot(data["Timestamp"], data["Thrust"], label="Thrust")
plt.title("Thrust")
plt.ylabel("Newtons")
plt.legend()
plt.xticks([])

plt.tight_layout()
plt.show()