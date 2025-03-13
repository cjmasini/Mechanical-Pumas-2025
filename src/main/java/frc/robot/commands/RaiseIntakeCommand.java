package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class RaiseIntakeCommand extends Command {

    private final IntakeSubsystem intakeSubsystem;

    /**
     * Command for lowering the intake
     *
     * @param intakeSubsystem
     *                        The game subsystem.
     */
    public RaiseIntakeCommand(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute() {
        this.intakeSubsystem.setWinchSpeed(.5);
    }

    @Override
    public boolean isFinished() {
        return false;//this.intakeSubsystem.isRaised();
    }

    @Override
    public void end(boolean interrupted) {
        this.intakeSubsystem.setWinchSpeed(0);
    }
}
