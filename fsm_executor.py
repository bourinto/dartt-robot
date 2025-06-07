"""Execution framework for the robot finite state machine."""

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

from tools import *

# global variables
f = fsm.FiniteStateMachine()  # finite state machine


# functions (actions of the fsm)
def doWait():
    """Idle state."""
    pass


def doMove():
    """Move forward while keeping heading."""
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


def doMove_lidar():
    """Move forward using lidar-based wall following."""
    left_dist_pid.reset()
    right_dist_pid.reset()

    d = 401
    while d > 400:

        lidar_data = np.array(mybot.lidar.get_scan(debug=False)[0]['scan'])[:, 1:]
        left_line, left_treshold = get_line(lidar_data, 'left')
        right_line, right_treshold = get_line(lidar_data, 'right')

        if len(left_line):
            corr = - left_dist_pid.compute(left_line[0])
            if abs(corr) < 2:
                corr = heading_pid.compute(left_line[1])

            if abs(left_line[1] - 90) < 10:
                d = get_lidar_distance(lidar_data, 'front')

        elif len(right_line):
            corr = - right_dist_pid.compute(right_line[0])
            if abs(corr) < 2:
                corr = heading_pid.compute(right_line[1])

            if abs(right_line[1] - 90) < 10:
                d = get_lidar_distance(lidar_data, 'front')

        else:
            corr = 0
            d = get_lidar_distance(lidar_data, 'front')

        mybot.powerboard.set_speed(spd_lidar - corr, spd_lidar + corr)

        time.sleep(0.1)

    mybot.powerboard.set_speed(0, 0)
    time.sleep(0.5)
    return "obstacle"


def doStop():
    """Stop when an obstacle is detected."""
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
    """Rotate 180 degrees."""
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
    """Rotate to the current target heading."""
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


def doRotate_lidar():
    """Rotate until lidar does not detect a wall on the side."""
    lidar_data = np.array(mybot.lidar.get_scan(debug=False)[0]['scan'])[:, 1:]
    if get_lidar_distance(lidar_data, 'right') > 500:
        mybot.powerboard.set_speed(50, -50)
        time.sleep(1)
        return "end_rotation"

    elif get_lidar_distance(lidar_data, 'left') > 500:
        mybot.powerboard.set_speed(-50, 50)
        time.sleep(1)
        return "end_rotation"

    else:
        return "end"


def doFinish():
    """Gracefully stop the robot."""
    print("End of the program")
    mybot.powerboard.stop()  # stop motors
    mybot.lidar.fullstop()  # clean stop of the RP Lidar
    mybot.end()  # clean end of the robot mission


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-file", type=str, required=True, help="Path to fsm.txt file")
    args = parser.parse_args()

    f.load_fsm_from_file(args.file)

    mybot = drv.DartV2DriverV3()  # create the virtual robot
    mybot.lidar.start()  # start the lidar
    time.sleep(1.)

    num = 1  # rear sonar
    dmax = 2.0  # maximum distance in meters
    mybot.sonars.set_dist_max(5, dmax)  # set dmax for Continuous Synchronous mode
    mybot.sonars.set_dist_max(num, dmax)  # set dmax for sonar num=3
    mybot.sonars.set_mode(num, 2)  # start the measurement

    t0 = 0
    kp = 50
    spd = 120
    spd_lidar = 50
    inertia_distance = 20  # cm (obtained with measure_inertia.py)
    distance_to_stop = 31  # cm
    h0 = getHeading(mybot.imu.read_mag_raw())

    left_dist_pid = PID(1 / 22., 0, -1 / 8.)
    left_dist_pid.target = -370

    right_dist_pid = PID(1 / 22., 0, -1 / 8.)
    right_dist_pid.target = 370

    heading_pid = PID(-1, 0, 0)
    heading_pid.target = 90

    f.exe()
