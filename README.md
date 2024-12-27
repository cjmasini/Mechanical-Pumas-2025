# Team 6616 Mechanical Pumas Max Swerve Starter Code
This is a template designed to be used with REV's MaxSwerve Modules. Originally based on REV's MaxSwerveTemplate, this codebase provides several key upgrades, the most notable of which is the addition of automatic field-centric orientation, where the robot will automically turn and face the indicated direction (Forward, Backward, Left, or Right) without additional human input. This codebase also adds fine-tuned robot-oriented control using the DPAD and refactors most of the MaxSwerve codebase, unifying code style conventions and removing or simplifying uneccessary or overly complicated code. 

## Getting Started
Before deploying this code to your robot, open the `Constants.java` file. This file contains all of the fields that are specific to your robot and need to be updated before deploying. Each field is marked by a "TODO" comment explaining what needs to be updated before running the code

## Default Control Scheme
This template is currently set up to use an XBox Controller, but could be easily modified for your controller of choice. Our default control scheme uses the left joystick to control field-oriented translation of the robot, and the x-axis of right joystick to control orientation of the robot, same as the original MaxSwerve template. In addition, holding the right trigger and any of the A/B/X/Y face buttons will cause the robot to auto orient itself to the direction corresponding with the face button. The robot will continue turning until either the button has been released or it has reached its desired heading. Finally, the DPAD on the controller is configured to move slowly (20% speed) in a robot centric manner, to enable fine tuned positional adjustments when looking through a camera positioned on the robot. 

## Example Command
For teams new to the Command Based Programming Paradigm, we have included a sample `ExampleMotorCommand` that sets a motor in the same  way it could be set for running a shooter or intake. The `GameSubsystem` class uses this motor and is set up similarly to how you would set up a subsystem for the superstruture for any given game.
