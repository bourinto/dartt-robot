"""Utility to estimate the robot inertia by measuring its stopping distance."""

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

if __name__ == "__main__":
    mybot = drv.DartV2DriverV3()  # create the virtual robot

    wheel_perimeter = np.pi * 0.125  # m
    mass = 8  # kg

    spd = 120
    duration = 1.0

    mybot.powerboard.set_speed(spd, spd)
    time.sleep(duration)

    odo_left0, odo_right0 = mybot.encoders.read_encoders()
    mybot.powerboard.set_speed(0, 0)

    odo_left, odo_right = -1, -1
    t = 0  # s
    dt = 0.05  # s

    while [odo_left, odo_right] != mybot.encoders.read_encoders():
        odo_left, odo_right = mybot.encoders.read_encoders()
        time.sleep(dt)
        t += dt

    delta_tick = odo_left0 - odo_left  # 65535
    d = delta_tick / 300 * wheel_perimeter  # m

    print(f"Distance to stop {d:.3f} m       Time to stop {t:.3f} s")


    inertia = 2 * d * mass / t  # m.kg/s
    print(f"Inertia : {inertia:.3f} kg.m/s")

    mybot.end()  # clean end of the robot mission
