# 2018 FRC Robot Code
Team 4237's FRC robot code for the 2018 robot: "Milkman". Milkman's code is written in Java and is based on the WPILib control system.

## Packages
### org.usfirst.team4237.robot.components

This package contains the class files for each primary component used on the robot.
Each component's class implements the Component interface to ensure that each class
 includes a method for printing information for testing.

It includes:
* Drivetrain
* Elevator
* Gripper
* Light Ring

### org.usfirst.team4237.robot.control

This package contains classes for use in controlling the robot.
The most important class in this package is the Xbox class,
which inherits from WPILib's Joystick class and makes it slightly more usable
when writing code by adding constants for each button.
This package also contains classes for both the driver's controller
and the operator's controller, limited to one instance of each.

### org.usfirst.team4237.robot.network
This package contains classes for use in communicating between the robot,
the driver station's autonomous selector, and a raspberry pi.

### org.usfirst.team4237.robot.sensors
This package contains classes for sensors used on the robot,
as well as some wrapper classes for other sensors.

### org.usfirst.team4237.robot.util
This package contains utility classes for use with the color sensor and debugging.

### org.usfirst.team4237.robot.vision
This package contains classes used for identifying reflective tape targets on the field
with computer vision on a raspberry pi for the purpose of aligning the robot to accomplish a goal.
This year it was unused in favor of faster preprogrammed autonomous routines.

### org.usfirst.team4237.robot
This package contains the main classes used to control the robot in both of its main modes, teleoperated and autonomous.

# Code Highlights

## Autonomous
The Autonomous class controls the robot during the first 15 seconds of each match.
In order to decide the optimal routine for autonomous, the robot is sent a Plan A, a Plan B, and a Plan C.
The robot then uses this data, as well as the conditions of the field to decide the best autonomous mode to run.

For example: The robot is starting on the right side of the field on the red alliance, and the autonomous plan is as follows
 * Plan A: Drop cube on right side of scale.
 * Plan B: Drop cube on right side of switch.
 * Plan C: Cross auto-line.

In this case, if the robot sees that the scale is blue on the right, it will check to see if Plan B is viable.
If it is, it will follow through on the plan. If it isn't, it will move on to Plan C, where it will either follow through or do nothing.

## AutoSelect4237

AutoSelect4237 is Team 4237's custom autonomous mode selector programmed in Python.

### On the driver station

Mode selection is done through an intuitive UI made using the PySide, a Qt wrapper from Python.
The dashboard interacts with the robot through UDP, sending data encoded in a JSON string.
Using UDP as opposed to NetworkTables has the advantage of future-proofing the code
since it will not need to be changed when NetworkTables changes.

### On the robot

Using the _AutoSelect4237_ class, the robot is able to continuously listen for data being sent.
This class inherits the _Thread_ class, allowing it to run in the background uninterrupted by other code.
Data received from the dashboard is decoded by the _AutoSelect4237_ class into an instance of _AutoSelect4237Data_.
which allows for easy access to the data sent by the dashboard.