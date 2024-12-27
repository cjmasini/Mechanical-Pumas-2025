package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.GameSubsystem;

/**
 * Command for note intake
 */
public class ExampleMotorCommand extends SequentialCommandGroup 
{

  private final GameSubsystem gameSubsystem;
  
  /**
   * Command for intaking notes
   *
   * @param game_subsystem  The game subsystem.
   */
  public ExampleMotorCommand(GameSubsystem game_subsystem)
  {
    this.gameSubsystem = game_subsystem;

    Command setExampleMotorSpeed = new InstantCommand(() -> this.gameSubsystem.setExampleMotorSpeed(1), game_subsystem);
    WaitCommand oneSecondWait = new WaitCommand(1);
    this.addCommands(setExampleMotorSpeed, oneSecondWait);
    addRequirements(game_subsystem);
  }
}


  
