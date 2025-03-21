package frc.robot.commands;

import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CancelableSubsystemBase;

/**
 * Command for cancelling all game subsystem mechanisms
 */
public class CancelCommand extends Command {

  private final List<CancelableSubsystemBase> subsystems;

  /**
   * Cancel all game subsystem mechanisms
   *
   * @param gameSubsystem
   *          The launcher subsystem.
   */
  public CancelCommand(List<CancelableSubsystemBase> subsystems) {
    this.subsystems = subsystems;

    for (CancelableSubsystemBase subsystem : this.subsystems) {
      addRequirements(subsystem);
    }
  }

  // Called when the command is first initialized by the scheduler
  @Override
  public void initialize() {
    for (CancelableSubsystemBase subsystem : this.subsystems) {
      subsystem.cancel();
    }
  }

  @Override
  public void execute() {
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}