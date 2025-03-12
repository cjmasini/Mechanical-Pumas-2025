package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants.Level;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {
    private final ElevatorSubsystem elevatorSubsystem;
    private final CoralSubsystem coralSubsystem;
    private final IntakeSubsystem intakeSubsystem;

    private boolean coralObtained = false;

    /**
     * Command for intaking a coral
     * @param elevatorSubsystem 
     * @param coralSubsystem
     * @param intakeSubsystem
     */
    public IntakeCommand(ElevatorSubsystem elevatorSubsystem,
            CoralSubsystem coralSubsystem, IntakeSubsystem intakeSubsystem) {
        addRequirements(elevatorSubsystem, coralSubsystem, intakeSubsystem);
        this.elevatorSubsystem = elevatorSubsystem;
        this.coralSubsystem = coralSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.coralObtained = false;
    }

    /**
     * Lowers the elevator if it is not at the bottom
     * Expected behavior is that elevator is already down when intaking begins
     * @param elevatorSubsystem 
     * @param coralSubsystem
     * @param intakeSubsystem
     * @param canRangeSensor
     */
    @Override
    public void initialize() {
        if (!elevatorSubsystem.isAtLevel(Level.DOWN)) {
            elevatorSubsystem.setLevel(Level.DOWN);
            while (true) {
                if (elevatorSubsystem.isAtLevel(Level.DOWN)) {
                    break;
                }
            }
        }
    }

    /**
     * Sets belt and coral motor speeds to intake coral
     * Stops when coral is obtained
     * Lowers speed once coral is in the coral mechanism 
     */
    @Override
    public void execute() {
        if (!coralObtained && intakeSubsystem.getTOFDistanceInches() < 3) 
            coralObtained = true;

        if (coralObtained){
            coralSubsystem.setCoralMotorSpeed(.2);
            intakeSubsystem.setBeltSpeed(1);
        } else {
            coralSubsystem.setCoralMotorSpeed(.5);
            intakeSubsystem.setBeltSpeed(1);
        }
        
        
    }

    @Override
    public void end(boolean interrupted) {
        coralSubsystem.setCoralMotorSpeed(0);
        intakeSubsystem.setBeltSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return coralObtained && intakeSubsystem.getTOFDistanceInches() > 3;
    }
}
