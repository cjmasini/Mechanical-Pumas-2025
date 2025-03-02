package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class LowerIntakeCommand extends Command {

    private final IntakeSubsystem intakeSubsystem;

    /**
     * Command for lowering the intake
     *
     * @param intakeSubsystem
     *                        The game subsystem.
     */
    public LowerIntakeCommand(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        this.intakeSubsystem.setWinchSpeed(-1);
    }

    @Override
    public boolean isFinished() {
        return this.intakeSubsystem.isLowered();
    }

    @Override
    public void end(boolean interrupted) {
        this.intakeSubsystem.setWinchSpeed(0);
    }
}
