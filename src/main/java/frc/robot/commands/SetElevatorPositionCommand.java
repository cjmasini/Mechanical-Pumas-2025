package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants.Level;
import frc.robot.subsystems.ElevatorSubsystem;

/**
 * Move elevator to a specific position
 */
public class SetElevatorPositionCommand extends Command {

  private final ElevatorSubsystem elevatorSubsystem;
  private final Level targetLevel;

  /**
   * Command for setting the elevator to a specific position
   *
   * @param elevatorSubsystem
   *          The elevator subsystem.
   */
  public SetElevatorPositionCommand(Level targetLevel, ElevatorSubsystem elevatorSubsystem) {
    this.elevatorSubsystem = elevatorSubsystem;
    addRequirements(elevatorSubsystem);

    this.targetLevel = targetLevel;
  }

  @Override
  public void execute() {
    this.elevatorSubsystem.setLevel(targetLevel);
  }

  @Override
  public void end(boolean interrupted) {
    this.elevatorSubsystem.cancel();
  }
}
