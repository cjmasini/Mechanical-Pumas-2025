package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.CoralSubsystem;

/**
 * Command for scoring coral
 */
public class EjectCoralCommand extends SequentialCommandGroup {

  private final CoralSubsystem coralSubsystem;

  /**
   * Command for scoring coral
   *
   * @param coralSubsystem
   *          The game subsystem.
   */
  public EjectCoralCommand(CoralSubsystem coralSubsystem) {
    SmartDashboard.putString("test", "Eject");
    this.coralSubsystem = coralSubsystem;

    Command startCoralMotor = new InstantCommand(() -> this.coralSubsystem.setCoralMotorSpeed(0.5), coralSubsystem);
    WaitCommand oneSecondWait = new WaitCommand(0.5);
    Command stopCoralMotor = new InstantCommand(() -> this.coralSubsystem.setCoralMotorSpeed(0), coralSubsystem);
    this.addCommands(startCoralMotor, oneSecondWait, stopCoralMotor);
    addRequirements(coralSubsystem);
  }
}
