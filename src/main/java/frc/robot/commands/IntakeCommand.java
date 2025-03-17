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

<<<<<<< Updated upstream
    private static final double PULSE_INTERVAL = 0.05; // 50ms pulse interval
    private static final double DETECTION_THRESHOLD = 5; // Inches for TOF detection

    private static final double CORAL_FORWARD_SPEED = 0.25; // Forward speed
    private static final double CORAL_REVERSE_SPEED = -0.15; // Reverse speed after detection
    private static final double BELT_SPEED = 0.5; // Belt motor runs constantly until coral is detected
    private static final int REVERSE_PULSES = 8; // Number of reverse pulses
=======
    private static final double TIME_OF_PULSE = 0.04; // Seconds per pulse
    private static final double DETECTION_THRESHOLD = 5; // Inches for TOF detection

    private static final double CORAL_SPEED = 0.25; // Coral motor speed
    private static final double BELT_SPEED = 0.5;   // Belt motor speed (initially on)
    private static final double REVERSE_SPEED = -0.15; // Reverse coral motor speed after securing
>>>>>>> Stashed changes

    private boolean isCoralMotorOn = false;
    private boolean coralDetected = false;
    private boolean securingCoral = false;
<<<<<<< Updated upstream
    private int reversePulseCount = 0;

    public IntakeCommand(ElevatorSubsystem elevatorSubsystem,
                         CoralSubsystem coralSubsystem, 
=======
    private boolean reversing = false;
    private int reversePulsesRemaining = 7;

    public IntakeCommand(ElevatorSubsystem elevatorSubsystem,
                         CoralSubsystem coralSubsystem,
>>>>>>> Stashed changes
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
<<<<<<< Updated upstream
        reversePulseCount = 0;
=======
        reversing = false;
        reversePulsesRemaining = 10; // Reset reverse pulse count
>>>>>>> Stashed changes
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
<<<<<<< Updated upstream
=======
        intakeSubsystem.setBeltSpeed(BELT_SPEED);
        
>>>>>>> Stashed changes
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
<<<<<<< Updated upstream
        }

        // Pulse logic
        if (!coralDetected) {
            // Searching phase: Pulse forward every 0.05s
            if (!isCoralMotorOn && pulseTimer.hasElapsed(PULSE_INTERVAL)) {
                coralSubsystem.setCoralMotorSpeed(CORAL_FORWARD_SPEED);
                pulseTimer.reset();
                isCoralMotorOn = true;
            } else if (isCoralMotorOn && pulseTimer.hasElapsed(PULSE_INTERVAL)) {
=======
            reversing = true; // Start reversing the coral motor
        }

        if (reversing) {
            intakeSubsystem.setBeltSpeed(0);
        }

        // Pulse the coral motor while securing coral
        if (securingCoral || !coralDetected) {
            if (!isCoralMotorOn && pulseTimer.hasElapsed(TIME_OF_PULSE)) {
                // Turn coral motor ON
                coralSubsystem.setCoralMotorSpeed(CORAL_SPEED);
                pulseTimer.reset();
                isCoralMotorOn = true;
            } else if (isCoralMotorOn && pulseTimer.hasElapsed(TIME_OF_PULSE)) {
                // Turn coral motor OFF
>>>>>>> Stashed changes
                coralSubsystem.setCoralMotorSpeed(0);
                pulseTimer.reset();
                isCoralMotorOn = false;
            }
<<<<<<< Updated upstream
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
=======
        }

        // After coral is secured, pulse the coral motor in reverse 10 times
        if (reversing) {
            if (reversePulsesRemaining > 0) {
                if (!isCoralMotorOn && pulseTimer.hasElapsed(TIME_OF_PULSE)) {
                    // Turn coral motor ON in reverse
                    coralSubsystem.setCoralMotorSpeed(REVERSE_SPEED);
                    pulseTimer.reset();
                    isCoralMotorOn = true;
                } else if (isCoralMotorOn && pulseTimer.hasElapsed(TIME_OF_PULSE)) {
                    // Turn coral motor OFF
                    coralSubsystem.setCoralMotorSpeed(0);
                    pulseTimer.reset();
                    isCoralMotorOn = false;
                    reversePulsesRemaining--; // Decrease the reverse pulse count
                }
            } else {
                // Stop reversing after 10 pulses
                coralSubsystem.setCoralMotorSpeed(0);
                reversing = false; // End the reverse pulsing
            }
>>>>>>> Stashed changes
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
<<<<<<< Updated upstream
        // Stop when the coral is detected and then no longer detected,
        // and after completing 8 reverse pulses
        return coralDetected && !securingCoral && reversePulseCount >= REVERSE_PULSES;
=======
        // Stop when the coral is detected and then no longer detected, and after 10 reverse pulses
        return coralDetected && !securingCoral && reversePulsesRemaining == 0;
>>>>>>> Stashed changes
    }
}
