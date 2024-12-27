// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.CancelCommand;
import frc.robot.commands.ExampleMotorCommand;
import frc.robot.commands.MoveCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.GameSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
  private final DriveSubsystem drivetrain = new DriveSubsystem();

  // The robot's game specific mechanisms are definded in this subsystem
  private final GameSubsystem gameSubsystem = new GameSubsystem();
  
  private final CommandXboxController driverXbox = new CommandXboxController(0);
  
  private final SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {

    MoveCommand moveCommand = new MoveCommand(this.drivetrain, driverXbox);
    drivetrain.setDefaultCommand(moveCommand);

    configureBindings();

    // Register all commands necessary for auto with the name set in path planner
    NamedCommands.registerCommand("runExampleMotor", new InstantCommand(() -> gameSubsystem.setExampleMotorSpeed(1)));
    NamedCommands.registerCommand("stopExampleMotor", new InstantCommand(() -> gameSubsystem.setExampleMotorSpeed(0)));

    SmartDashboard.putData(autoChooser);

  }

  private void configureBindings() {
    // Run Example Motor Command with a timeout of 10s
    // Must only check for A/B/X/Y when right trigger is not pressed down to prevent triggering 
    // while trying to automatically orient
    ExampleMotorCommand exampleMotorCommand = new ExampleMotorCommand(gameSubsystem);
    driverXbox.a().and(driverXbox.rightTrigger().negate()).onTrue(exampleMotorCommand.withTimeout(10));

    // Right trigger is used to cancel other commands and as a modifier for face buttons
    CancelCommand cancelCommand = new CancelCommand(gameSubsystem);
    driverXbox.rightTrigger().onTrue(cancelCommand.withTimeout(10));

    // Manually re-zero the gyro if it gets off during competition
    // With the pigeon Gyro, we only needed to do this because of user error in setup
    InstantCommand resetGyro = new InstantCommand(() -> this.drivetrain.zeroHeading());
    driverXbox.rightStick().and(driverXbox.rightTrigger()).onTrue(resetGyro);

    // Toggle drive mode command, currently disabled as we did not find it necessary
    // InstantCommand toggleDriveMode = new InstantCommand(() -> moveCommand.toggleFieldReletive());
    // driverXbox.y().and(driverXbox.rightTrigger().negate()).onTrue(toggleDriveMode);
  }

  // Return the auto selected in smart dashboard
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}