import os
import sys
import numpy as np
from heading_calibration import getHeading

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

kp = 100

if __name__ == "__main__":
    mybot = drv.DartV2DriverV3()  # create the virtual robot

    num = 1  # rear sonar
    dmax = 2.0  # maximum distance in meters
    mybot.sonars.set_dist_max(5, dmax)  # set dmax for Continuous Synchronous mode
    mybot.sonars.set_dist_max(num, dmax)  # set dmax for sonar num=3
    mybot.sonars.set_mode(num, 2)  # start the measurement

    spd = 120
    inertia_distance = 20  # cm (obtained with measure_inertia.py)
    distance_to_stop = 25  # cm

    h0 = getHeading(mybot.imu.read_mag_raw())

    while mybot.sonars.read_front() > distance_to_stop + inertia_distance:
        head = getHeading(mybot.imu.read_mag_raw())
        dh = 2 * np.arctan(np.tan((head - h0) / 2))
        corr = dh * kp
        mybot.powerboard.set_speed(spd - corr, spd + corr)
        time.sleep(0.1)  # 10 Hz

    mybot.powerboard.set_speed(0, 0)
    h0 += np.pi

    corr = np.inf

    while abs(corr) > 15:
        head = getHeading(mybot.imu.read_mag_raw())
        dh = 2 * np.arctan(np.tan((head - h0) / 2))
        corr = dh * kp
        mybot.powerboard.set_speed(-corr, +corr)
        time.sleep(0.1)  # 10 Hz

    mybot.powerboard.set_speed(0, 0)

    time.sleep(1)
    print(f"End distance to the wall : {mybot.sonars.read_front()} cm.")

    mybot.powerboard.stop()  # stop motors
    mybot.end()  # clean end of the robot mission
