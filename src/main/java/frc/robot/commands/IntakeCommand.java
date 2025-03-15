package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants.Level;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {
    private final ElevatorSubsystem elevatorSubsystem;
    private final CoralSubsystem coralSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final Timer reverseTimer;

    private boolean coralObtained = false;

    /**
     * Command for intaking a coral
     * 
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
        this.reverseTimer = new Timer();
        this.coralObtained = false;
    }

    /**
     * Lowers the elevator if it is not at the bottom
     * Expected behavior is that elevator is already down when intaking begins
     * 
     * @param elevatorSubsystem
     * @param coralSubsystem
     * @param intakeSubsystem
     * @param canRangeSensor
     */
    @Override
    public void initialize() {
        // if (!elevatorSubsystem.isAtLevel(Level.DOWN)) {
        //     elevatorSubsystem.setLevel(Level.DOWN);
        //     while (true) {
        //         if (elevatorSubsystem.isAtLevel(Level.DOWN)) {
        //             break;
        //         }
        //     }
        // }
    }

    /**
     * Sets belt and coral motor speeds to intake coral
     * Stops when coral is obtained
     * Lowers speed once coral is in the coral mechanism
     */
    @Override
    public void execute() {
        intakeSubsystem.setBeltSpeed(.5);
        // if (!coralObtained && intakeSubsystem.getTOFDistanceInches() < 5) {
        //     // coralSubsystem.setCoralMotorSpeed(0);
        //     intakeSubsystem.setBeltSpeed(.2);
        //     coralObtained = true;
        //     reverseTimer.start();
        // }
        // if (reverseTimer.hasElapsed(.1)) {
        //     coralSubsystem.setCoralMotorSpeed(0);
        //     intakeSubsystem.setBeltSpeed(0);
        //     reverseTimer.start();
        // } else {
        //     coralSubsystem.setCoralMotorSpeed(.20);
        //     intakeSubsystem.setBeltSpeed(.5);
        // }
    }

    @Override
    public void end(boolean interrupted) {
        coralSubsystem.setCoralMotorSpeed(0);
        intakeSubsystem.setBeltSpeed(0);
        coralObtained = false;
    }

    @Override
    public boolean isFinished() {
        return true;//coralObtained && reverseTimer.hasElapsed(.3);
    }
}
