# DART Robot Control Programs

This repository contains control programs for the DART robot, a four-wheeled robot equipped with sensors (sonars,
odometers, inertial navigation unit, lidar) and actuators (motors, 7-segment display). These programs are designed to
control and interact with the robot for autonomous navigation and experimentation.

## Author

**Toméo BOURIN**  
Student at ENSTA Bretagne, specializing in Autonomous Robotics.

## Programs

- **`measure_inertia.py`**: Measures the inertia of the DART robot by analyzing its stopping behavior after movement.


- **`obstacle_detection_test.py`**: Tests the robot's obstacle detection capabilities by using the rear sonar sensor to
  move away from the wall and automatically stop at a specified distance.


- **`heading_calibration.py`**: Calibrates the robot's magnetometer by computing a transformation matrix based on
  magnetic field readings in known orientations *(see the "How to Calibrate the Compass" section below)*


- **`heading_control.py`**: Controls the robot to move forward while maintaining its initial heading. The robot stops at
  a specified safe distance from an obstacle and then performs a 180-degree turn to move away from it.

*More programs will be added to this repository in the future.*

## How to Calibrate the Compass

To calibrate the robot's compass using the `heading_calibration.py` script:

1. **Collect Data**: Place the robot facing North and run the script to record average magnetometer readings. Repeat
   this for East, South, and West orientations.


2. **Update Script**: Input the collected averages into the `north`, `east`, `south`, and `west` variables within the
   script.


3. **Compute Calibration and Use It**: The script will compute a transformation matrix to refine compass accuracy. You
   can then use the `getHeading(mag)` function in your programs to obtain calibrated heading readings.
