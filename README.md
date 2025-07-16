<div align="center">
<h1>Iron Tank 2025 Codes</h1>
The beta version of robot codes written for Sanya off-season FRC competition.<br><br>

</div>
This project contains the beta version of the control software for the Iron Tank 2025 robot, designed for the Sanya off-season FIRST Robotics Competition. These softwares are included in the version: 

* WPILib for robot control
* CTRE Phoenix for motor and actuator management
* Advantage Kit for logging and debugging
* Tuner FX for motor controller configuration and tuning
* Motion Magic for precise closed-loop control of mechanisms
* Differential Drive for smooth and efficient robot movement

The code is structured using command-based programming, with subsystems and commands organized for modular and efficient robot operation.

This code is about to be implemented on team Everybot 9994

## Known issues & To Do List

Coming soon

## Release Notes

### Overview
This is the first beta release of the control software built for Sanya off-season FIRST Robotics Competition and designed for implementation on team Everybot 9994
This version of code includes three subsystem, designed to control the robot's driving, intake, and climber.
The robot should be able to move to desired position, intake coral pieces, elevate it to desired height, and by that place the game piece on the coral stage. 
### Key Features

**1. Robot Framework (WPILib Command-Based)**
The robot code leverages WPILib’s command-based structure:
- Main robot lifecycle managed by Robot.java using TimedRobot.
- We uses RobotContainer for initialization and command scheduling.

**2. Drive Subsystem**
- This subsystem implements all core drivetrain functionality for the robot.
- The robot is controlled by selected buttons on an Xbox controller or 'joystick'.
- Currently only supports teleoperated or manual driving.

**3. Intake Subsystem**
- This subsystem uses CTRE Pheonix 6 for motor and encoder control, and AdvantageKit's Logger for telemetry.
- Same as other subsystems, constants in this subsystem are located in consts class, including hardware IDs, PID values, current limits, and tunable parameters.
- The subsystem is capable of running the roller motor to intake and eject coral game pieces, with methods to start or stop actions.
- **Dynamic Tuning:** The subsystem monitors changes in position constants and reconfigures motors when needed.

**4. Climber Subsystem**
- Controls a single TalonFX motor which controls the elevators.
- The subsystem achieve that by setting the climber motor to a specific voltage to raise or lower the climber.
- All subsystem log key information, this subsystem continuosly logs the climber motor's position, velocity, current, and voltage for diagnostics and monitoring.

## Credits

- [Hongfei Liu](https://github.com/AlltAWD)
- [Yifan Cao](https://github.com/MemoryArray)
- [Dale](https://github.com/Dale-D-A)
- [Xiqiu Yan](https://github.com/KuanYan666)

## Thanks to all contributors of this program
