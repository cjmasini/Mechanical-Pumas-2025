package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GameSubsystem;

/**
 * Command for note intake
 */
public class CancelCommand extends Command 
{

  private final GameSubsystem gameSubsystem;

  /**
   * Cancel all game subsystem mechanisms
   *
   * @param gameSubsystem  The launcher subsystem.
   */
  public CancelCommand(GameSubsystem gameSubsystem)
  {

    this.gameSubsystem = gameSubsystem;

    addRequirements(gameSubsystem);
  }

  // Called when the command is first initialized by the scheduler
  @Override
  public void initialize()
  {
    // Set all motors to stop
    this.gameSubsystem.setExampleMotorSpeed(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}
}