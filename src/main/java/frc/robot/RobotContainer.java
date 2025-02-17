// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ElevatorConstants.Level;
import frc.robot.commands.CancelCommand;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.EjectCoralCommand;
import frc.robot.commands.FallCommand;
import frc.robot.commands.LowerElevatorCommand;
import frc.robot.commands.MoveCommand;
import frc.robot.commands.RaiseElevatorCommand;
import frc.robot.commands.SetElevatorPositionCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CoralSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final DriveSubsystem drivetrain = new DriveSubsystem();

  private final CoralSubsystem coralSubsystem = new CoralSubsystem();

  private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();

  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

  private final CommandXboxController driverXbox = new CommandXboxController(0);

  private final SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    MoveCommand moveCommand = new MoveCommand(this.drivetrain, driverXbox);
    drivetrain.setDefaultCommand(moveCommand);

    configureBindings();

    // Register all commands necessary for auto with the name set in path planner
    NamedCommands.registerCommand("ejectCoral", Commands.startEnd(() -> {
      coralSubsystem.setCoralMotorSpeed(1);
    }, () -> {
      coralSubsystem.setCoralMotorSpeed(0);
    }, coralSubsystem).withTimeout(1));

    SmartDashboard.putData(autoChooser);
  }

  private void configureBindings() {
    // Eject coral onto reef
    EjectCoralCommand ejectCoralCommand = new EjectCoralCommand(coralSubsystem);
    driverXbox.rightBumper().and(driverXbox.rightTrigger().negate()).onTrue(ejectCoralCommand.withTimeout(10));

    // Climb the cage while button is pressed
    ClimbCommand climbCommand = new ClimbCommand(climbSubsystem);
    driverXbox.a().whileTrue(climbCommand);

    // Slowly lower after climbing the cage
    FallCommand fallCommand = new FallCommand(climbSubsystem);
    driverXbox.b().whileTrue(fallCommand);

    // Raise elevator while button is pressed
    RaiseElevatorCommand raiseElevatorCommand = new RaiseElevatorCommand(elevatorSubsystem);
    driverXbox.y().whileTrue(raiseElevatorCommand);

    // Lower elevator while button is pressed
    LowerElevatorCommand lowerElevatorCommand = new LowerElevatorCommand(elevatorSubsystem);
    driverXbox.x().whileTrue(lowerElevatorCommand);

    // Set the elevator to a specific position
    Level level = Level.L2;
    SetElevatorPositionCommand setElevatorPositionCommand = new SetElevatorPositionCommand(level, elevatorSubsystem);
    driverXbox.leftTrigger().onTrue(setElevatorPositionCommand);

    // Right trigger is used to cancel other commands and as a modifier for face
    // buttons
    CancelCommand cancelCommand = new CancelCommand(
        List.of(drivetrain, coralSubsystem, climbSubsystem, elevatorSubsystem));
    driverXbox.rightTrigger().onTrue(cancelCommand.withTimeout(10));

    // Manually re-zero the gyro if it gets off during competition
    // With the pigeon Gyro, we only needed to do this because of user error in
    // setup
    InstantCommand resetGyro = new InstantCommand(() -> this.drivetrain.zeroHeading());
    driverXbox.rightStick().and(driverXbox.rightTrigger()).onTrue(resetGyro);

    // Toggle drive mode command, currently disabled as we did not find it necessary
    // InstantCommand toggleDriveMode = new InstantCommand(() ->
    // moveCommand.toggleFieldReletive());
    // driverXbox.y().and(driverXbox.rightTrigger().negate()).onTrue(toggleDriveMode);
  }

  // Return the auto selected in smart dashboard
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}