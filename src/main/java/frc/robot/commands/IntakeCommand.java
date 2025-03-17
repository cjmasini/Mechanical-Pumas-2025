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
    private final Timer pulseTimer;

    private static final double PULSE_INTERVAL = 0.05; // 50ms pulse interval
    private static final double DETECTION_THRESHOLD = 5; // Inches for TOF detection

    private static final double CORAL_FORWARD_SPEED = 0.25; // Forward speed
    private static final double CORAL_REVERSE_SPEED = -0.15; // Reverse speed after detection
    private static final double BELT_SPEED = 0.5; // Belt motor runs constantly until coral is detected
    private static final int REVERSE_PULSES = 8; // Number of reverse pulses

    private boolean isCoralMotorOn = false;
    private boolean coralDetected = false;
    private boolean securingCoral = false;
    private int reversePulseCount = 0;

    public IntakeCommand(ElevatorSubsystem elevatorSubsystem,
                         CoralSubsystem coralSubsystem, 
                         IntakeSubsystem intakeSubsystem) {
        addRequirements(elevatorSubsystem, coralSubsystem, intakeSubsystem);
        this.elevatorSubsystem = elevatorSubsystem;
        this.coralSubsystem = coralSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.pulseTimer = new Timer();
    }

    @Override
    public void initialize() {
        coralDetected = false;
        securingCoral = false;
        isCoralMotorOn = false;
        reversePulseCount = 0;
        pulseTimer.reset();
        pulseTimer.start();

        // Ensure the elevator is at the bottom before starting
        if (!elevatorSubsystem.isAtLevel(Level.DOWN)) {
            elevatorSubsystem.setLevel(Level.DOWN);
        }

        // Start belt motor (runs constantly until coral is detected)
        intakeSubsystem.setBeltSpeed(BELT_SPEED);
    }

    @Override
    public void execute() {
        // Wait for the elevator to reach the bottom before intaking
        if (!elevatorSubsystem.isAtLevel(Level.DOWN)) {
            return; // Don't start pulsing until the elevator is down
        }

        double currentTOF = intakeSubsystem.getTOFDistanceInches();

        // Check if coral is detected for the first time
        if (!coralDetected && currentTOF < DETECTION_THRESHOLD) {
            coralDetected = true;
            securingCoral = true;
            pulseTimer.reset();
            intakeSubsystem.setBeltSpeed(0); // Stop belt motor once coral is detected
        }

        // Check if coral has been fully secured (no longer seen by TOF sensor)
        if (securingCoral && currentTOF > DETECTION_THRESHOLD) {
            securingCoral = false; // Coral is no longer detected, stop the command
        }

        // Pulse logic
        if (!coralDetected) {
            // Searching phase: Pulse forward every 0.05s
            if (!isCoralMotorOn && pulseTimer.hasElapsed(PULSE_INTERVAL)) {
                coralSubsystem.setCoralMotorSpeed(CORAL_FORWARD_SPEED);
                pulseTimer.reset();
                isCoralMotorOn = true;
            } else if (isCoralMotorOn && pulseTimer.hasElapsed(PULSE_INTERVAL)) {
                coralSubsystem.setCoralMotorSpeed(0);
                pulseTimer.reset();
                isCoralMotorOn = false;
            }
        } else if (reversePulseCount < REVERSE_PULSES) {
            // Securing phase: Pulse in reverse 8 times
            if (!isCoralMotorOn && pulseTimer.hasElapsed(PULSE_INTERVAL)) {
                coralSubsystem.setCoralMotorSpeed(CORAL_REVERSE_SPEED);
                pulseTimer.reset();
                isCoralMotorOn = true;
            } else if (isCoralMotorOn && pulseTimer.hasElapsed(PULSE_INTERVAL)) {
                coralSubsystem.setCoralMotorSpeed(0);
                pulseTimer.reset();
                isCoralMotorOn = false;
                reversePulseCount++;
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Ensure everything stops at the end
        intakeSubsystem.setBeltSpeed(0);
        coralSubsystem.setCoralMotorSpeed(0);
        pulseTimer.stop();
    }

    @Override
    public boolean isFinished() {
        // Stop when the coral is detected and then no longer detected,
        // and after completing 8 reverse pulses
        return coralDetected && !securingCoral && reversePulseCount >= REVERSE_PULSES;
    }
}
