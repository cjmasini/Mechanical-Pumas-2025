package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants.Level;
import frc.robot.subsystems.ElevatorSubsystem;

/**
 * Move elevator to a specific position
 */
public class AutoSetElevatorLevelCommand extends Command {

  private final ElevatorSubsystem elevatorSubsystem;
  private final SendableChooser<Level> levelChooser;

  /**
   * Command for setting the elevator to whatever is selected in Elastic
   *
   * @param levelChooser
   *                          The chooser for the user-selected level
   * @param elevatorSubsystem
   *                          The elevator subsystem.
   */
  public AutoSetElevatorLevelCommand(SendableChooser<Level> levelChooser, ElevatorSubsystem elevatorSubsystem) {
    this.elevatorSubsystem = elevatorSubsystem;
    addRequirements(elevatorSubsystem);

    this.levelChooser = levelChooser;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    this.elevatorSubsystem.setLevel(levelChooser.getSelected());
  }

  @Override
  public boolean isFinished() {
    return this.elevatorSubsystem.isAtLevel(levelChooser.getSelected());
  }

  @Override
  public void end(boolean interrupted) {
    this.elevatorSubsystem.cancel();
  }
}
