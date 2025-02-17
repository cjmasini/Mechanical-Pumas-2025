package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

/**
 * Command for climbing cage
 */
public class ClimbCommand extends Command {

  private final ClimbSubsystem climbSubsystem;

  /**
   * Command for climbing the cage
   * Scheduled to run only when while the climb button is pressed down
   *
   * @param coralSubsystem
   *          The game subsystem.
   */
  public ClimbCommand(ClimbSubsystem climbSubsystem) {
    this.climbSubsystem = climbSubsystem;
    addRequirements(climbSubsystem);
  }

  @Override
  public void execute() {
    this.climbSubsystem.setClimbMotorSpeed(1);
  }

  @Override
  public void end(boolean interrupted) {
    this.climbSubsystem.setClimbMotorSpeed(0);
  }
}
