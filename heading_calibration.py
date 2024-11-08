import os
import sys
import numpy as np

# access to the drivers
sys.path.append(os.path.join(os.path.dirname(__file__), '../git'))
try:
    import dartv2_drivers_v3.drivers_v3 as drv
except:
    print('could not import dartv2_drivers_v3.drivers_v3, try adding .. to sys.path')
    # modify the end of the following line to point to dartv2_drivers_v3 folder
    sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
import dartv2_drivers_v3.drivers_v3 as drv
import time

"""Obtained value"""
north = [13097.10, 4954.18]
east = [-6721.48, 19657.70]
south = [-26230.05, 4899.70]
west = [-6564.03, -9836.67]

X = np.array([north, east, south, west])
X_augmented = np.hstack([X, np.ones((X.shape[0], 1))])
Y = np.array([[1, 0], [0, 1], [-1, 0], [0, -1]])
P, _, _, _ = np.linalg.lstsq(X_augmented, Y, rcond=None)


def getHeading(mag):
    x = np.array([mag[0], mag[1], 1])
    y = x @ P
    return -np.arctan2(y[0], y[1]) + np.pi / 2


if __name__ == "__main__":
    mybot = drv.DartV2DriverV3()  # create the virtual robot

    mag_x = 0
    mag_y = 0
    n = 60
    for i in range(n):
        mag = mybot.imu.read_mag_raw()
        mag_x += mag[0]
        mag_y += mag[1]
        head = mybot.imu.heading_raw_deg(mag[0], mag[1])
        print(i)
        time.sleep(1)

    print("avgmx= %.2f, avgmy= %.2f" % (mag_x / n, mag_y / n))

    mybot.powerboard.stop()  # stop motors
    mybot.end()  # clean end of the robot mission
