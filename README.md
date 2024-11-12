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


- **`fsm.py`**: Reads and processes a finite state machine (FSM) description from a `.txt` file *(see the "How to write
  a FSM.txt file" section below)*.


- **`fsm_executor.py`**: Contains all control commands and executes the finite state machine (FSM) for the DART robot *(
  see the "How to use fsm_executor.py" section below)*.

*More programs will be added to this repository in the future.*

## How to Calibrate the Compass

To calibrate the robot's compass using the `heading_calibration.py` script:

1. **Collect Data**: Place the robot facing North and run the script to record average magnetometer readings. Repeat
   this for East, South, and West orientations.


2. **Update Script**: Input the collected averages into the `north`, `east`, `south`, and `west` variables within the
   script.


3. **Compute Calibration and Use It**: The script will compute a transformation matrix to refine compass accuracy. You
   can then use the `getHeading(mag)` function in your programs to obtain calibrated heading readings.


## How to Write a `FSM.txt` File

To define the finite state machine (FSM) for the DART robot, create a `.txt` file with the following structure and
sections:

1. **States**: Define all possible states of the FSM. Each state represents a unique behavior or action the robot can
   take.

    ```
    ----- States
    Idle
    Stop
    Rotate
    Move
    End
    ```

2. **Transitions**: Define all possible transitions between states. Each transition requires four elements in the
   following format:  
   `[Current State] [Next State] [Event Trigger] [Action Function]`

   This structure indicates the action taken when the robot moves from one state to another due to a specific event.

    ```
    ----- Transitions
    Idle Idle wait doWait
    Idle Move go doMove
    Move Idle wait doWait
    Move Stop obstacle doStop
    Stop Move clear doMove
    Stop Rotate rotate doRotate
    Stop End end doFinish
    Rotate Stop end_rotation doStop
    ```

3. **Events**: List all events that can trigger state transitions. Events can be specific sensor detections, commands,
   or other conditions that the robot monitors.

    ```
    ----- Events
    wait
    go
    obstacle
    clear
    rotate
    end_rotation
    end
    ```

4. **Start State**: Specify the initial state of the FSM when the program begins.

    ```
    ---- Start State
    Idle
    ```

5. **Start Event**: Specify the initial event that triggers the FSM’s execution from the start state.

    ```
    ---- Start Event
    go
    ```

6. **End State**: Specify the final state where the FSM stops execution.

    ```
    ---- End State
    End
    ```

Be sure to align the states, transitions, and events with the Actions Functions defined in `fsm_executor.py`.

## How to Use `fsm_executor.py`

To execute the FSM for the DART robot, follow these steps:

1. Place your `FSM.txt` file in the same directory as `fsm_executor.py`.

2. Open a command prompt or terminal window.
3. Run the following command:

    ```bash
    python3 fsm_executor.py -file FSM.txt
    ```

This will load the FSM configuration from `FSM.txt` and execute the state-based actions.
