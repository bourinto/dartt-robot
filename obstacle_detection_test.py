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
    print("\n\n")

    num = 2  # rear sonar
    dmax = 2.0  # maximum distance in meters
    mybot.sonars.set_dist_max(5, dmax)  # set dmax for Continuous Synchronous mode
    mybot.sonars.set_dist_max(num, dmax)  # set dmax for sonar num=3
    mybot.sonars.set_mode(num, 2)  # start the measurement

    spd = 50
    inertia_distance = 5  # cm
    distance_to_stop = 90  # cm

    mybot.powerboard.set_speed(spd, spd)

    while mybot.sonars.read_rear() < distance_to_stop - inertia_distance:
        time.sleep(0.05)

    mybot.powerboard.set_speed(0, 0)

    time.sleep(1)
    print(f"End distance to the wall : {mybot.sonars.read_rear()} cm.")

    mybot.end()  # clean end of the robot mission
