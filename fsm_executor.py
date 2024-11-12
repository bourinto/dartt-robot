import fsm
import argparse
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

# global variables
f = fsm.fsm()  # finite state machine


# functions (actions of the fsm)
def doWait():
    pass


def doMove():
    global t0

    while True:
        head = getHeading(mybot.imu.read_mag_raw())
        dh = 2 * np.arctan(np.tan((head - h0) / 2))
        corr = dh * kp
        mybot.powerboard.set_speed(spd - corr, spd + corr)
        time.sleep(0.1)  # 10 Hz

        if mybot.sonars.read_front() < distance_to_stop + inertia_distance:
            t0 = time.time()
            return "obstacle"


def doStop():
    global h0
    mybot.powerboard.set_speed(0, 0)

    while True:
        if mybot.sonars.read_front() > distance_to_stop + inertia_distance:
            return "clear"

        elif mybot.sonars.read_left() > 50:
            h0 -= np.pi / 2
            return 'rotate'

        elif mybot.sonars.read_right() > 50:
            h0 += np.pi / 2
            return 'rotate'

        elif time.time() - t0 > 5:
            return 'end'

        time.sleep(0.1)  # 10 Hz


def doRotate180():
    global t0, h0
    corr = 31
    h0 += np.pi

    mybot.powerboard.set_speed(0, 0)

    while abs(corr) > 30:
        head = getHeading(mybot.imu.read_mag_raw())
        dh = 2 * np.arctan(np.tan((head - h0) / 2))
        corr = dh * kp
        mybot.powerboard.set_speed(-corr, +corr)
        time.sleep(0.1)  # 10 Hz

    t0 = time.time()
    return 'end_rotation'


def doRotate():
    global t0, h0
    corr = 31
    while abs(corr) > 30:
        head = getHeading(mybot.imu.read_mag_raw())
        dh = 2 * np.arctan(np.tan((head - h0) / 2))
        corr = dh * kp
        mybot.powerboard.set_speed(-corr, +corr)
        time.sleep(0.1)  # 10 Hz

    t0 = time.time()
    return 'end_rotation'


def doFinish():
    print("End of the programm")
    mybot.powerboard.stop()  # stop motors
    mybot.end()  # clean end of the robot mission


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-file", type=str, required=True, help="Path to fsm.txt file")
    args = parser.parse_args()

    f.load_fsm_from_file(args.file)

    mybot = drv.DartV2DriverV3()  # create the virtual robot

    num = 1  # rear sonar
    dmax = 2.0  # maximum distance in meters
    mybot.sonars.set_dist_max(5, dmax)  # set dmax for Continuous Synchronous mode
    mybot.sonars.set_dist_max(num, dmax)  # set dmax for sonar num=3
    mybot.sonars.set_mode(num, 2)  # start the measurement

    t0 = 0
    kp = 50
    spd = 120
    inertia_distance = 20  # cm (obtained with measure_inertia.py)
    distance_to_stop = 31  # cm
    h0 = getHeading(mybot.imu.read_mag_raw())

    f.exe()
