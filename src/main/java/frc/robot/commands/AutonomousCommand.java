package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.CoralSubsystem;

/**
 * Autonomous sequential command group
 */
public class AutonomousCommand extends SequentialCommandGroup {

  private final DriveSubsystem drivetrain;
  private final CoralSubsystem gameSubsystem;

  /**
   * Command for autonomous mode
   *
   * @param drivetrain
   *          The drivetrain
   * @param gameSubsystem
   *          The launcher subsystem.
   */
  public AutonomousCommand(DriveSubsystem driveSubsystem, CoralSubsystem gameSubsystem) {
    this.gameSubsystem = gameSubsystem;
    this.drivetrain = driveSubsystem;
    double speed = .5;

    WaitCommand waitCommand = new WaitCommand(1.5);
    Command moveForwardCommand = Commands
        .startEnd(() -> this.drivetrain.drive(speed, 0, 0, true), () -> this.drivetrain.drive(0, 0, 0, true))
        .withTimeout(1);
    Command runExampleMotorCommand = new RunCommand(this::runExampleMotor, driveSubsystem).withTimeout(2);
    WaitCommand waitCommand2 = new WaitCommand(1.5);
    Command stopExampleMotorCommand = new RunCommand(this::stopExampleMotor, driveSubsystem).withTimeout(2);

    this.addCommands(waitCommand, moveForwardCommand, runExampleMotorCommand, waitCommand2, stopExampleMotorCommand);
  }

  private void runExampleMotor() {
    this.drivetrain.drive(.5, 0, 0, true);
    this.gameSubsystem.setCoralMotorSpeed(0);

  }

  private void stopExampleMotor() {
    this.gameSubsystem.setCoralMotorSpeed(0);
  }
}
