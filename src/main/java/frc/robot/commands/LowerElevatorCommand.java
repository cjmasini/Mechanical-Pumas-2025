package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

/**
 * Manually move elevator down
 */
public class LowerElevatorCommand extends Command {

  private final ElevatorSubsystem elevatorSubsystem;

  /**
   * Command for manually lowering the elevator
   * Scheduled to run only when while the raise button is pressed down
   *
   * @param elevatorSubsystem
   *                          The elevator subsystem.
   */
  public LowerElevatorCommand(ElevatorSubsystem elevatorSubsystem) {
    this.elevatorSubsystem = elevatorSubsystem;
    addRequirements(elevatorSubsystem);
  }

  @Override
  public void execute() {
    this.elevatorSubsystem.setManualPower(-.25);
  }

  @Override
  public void end(boolean interrupted) {
    this.elevatorSubsystem.setManualPower(0);
  }
}
