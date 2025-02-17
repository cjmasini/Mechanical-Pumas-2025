package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ElevatorConstants.Level;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.CANIdConstants;

public class ElevatorSubsystem extends CancelableSubsystemBase {
    private final SparkMax elevatorMotor;
    private final SparkMax followerMotor;
    private final RelativeEncoder encoder;
    private final PIDController pidController;
    private final TrapezoidProfile.Constraints constraints;

    private TrapezoidProfile.State goalState;
    private TrapezoidProfile.State nextState;
    private final TrapezoidProfile profile;

    private Level targetLevel = Level.DOWN;
    private double targetInches = 0.0;
    double currentPos;

    public ElevatorSubsystem() {
        elevatorMotor = new SparkMax(CANIdConstants.LEFT_ELEVATOR_CAN_ID, MotorType.kBrushless);
        SparkMaxConfig elevatorMotorConfig = new SparkMaxConfig();
        elevatorMotorConfig.idleMode(IdleMode.kBrake);
        elevatorMotorConfig.smartCurrentLimit(40);
        elevatorMotorConfig.voltageCompensation(12.0);
        elevatorMotor.configure(elevatorMotorConfig, ResetMode.kResetSafeParameters, null);
        elevatorMotor.configure(elevatorMotorConfig, ResetMode.kResetSafeParameters, null);

        followerMotor = new SparkMax(CANIdConstants.RIGHT_ELEVATOR_CAN_ID, MotorType.kBrushless);
        SparkMaxConfig followerConfig = new SparkMaxConfig();
        followerConfig.follow(elevatorMotor, true);
        followerMotor.configure(followerConfig, null, null);

        encoder = elevatorMotor.getEncoder();

        pidController = new PIDController(
                ElevatorConstants.kP,
                ElevatorConstants.kI,
                ElevatorConstants.kD);
        pidController.setTolerance(ElevatorConstants.POSITION_TOLERANCE);

        constraints = new TrapezoidProfile.Constraints(
                ElevatorConstants.MAX_VELOCITY,
                ElevatorConstants.MAX_ACCELERATION);
        nextState = new TrapezoidProfile.State(0, 0);
        goalState = new TrapezoidProfile.State(0, 0);
        profile = new TrapezoidProfile(constraints);
    }

    @Override
    public void periodic() {
        currentPos = encoder.getPosition() / ElevatorConstants.COUNTS_PER_INCH;

        // Update the state for motion towards target over the next 20ms
        nextState = profile.calculate(0.020, nextState, goalState);

        if (getHeightInches() >= ElevatorConstants.MAX_POSITION) {
            elevatorMotor.set(0);
            pidController.reset();
        }

        double pidOutput = pidController.calculate(getHeightInches(), nextState.position);
        double ff = calculateFeedForward(nextState);

        double outputPower = MathUtil.clamp(
                pidOutput + ff,
                ElevatorConstants.MIN_POWER,
                ElevatorConstants.MAX_POWER);

        elevatorMotor.set(outputPower);

        // Update SmartDashboard
        logElevatorState();
    }

    public boolean isAtHeight(double targetHeightInches) {
        // Check if the elevator is within a small tolerance of the target height
        return pidController.atSetpoint() &&
                Math.abs(getHeightInches() - targetHeightInches) < ElevatorConstants.POSITION_TOLERANCE;
    }

    public void setLevel(Level level) {
        targetLevel = level;
        setPositionInches(level.getPosition());
    }

    public void setPositionInches(double inches) {
        targetInches = MathUtil.clamp(
                inches,
                ElevatorConstants.MIN_POSITION,
                ElevatorConstants.MAX_POSITION);

        // Update goal state for motion profile
        goalState = new TrapezoidProfile.State(targetInches, 0);
    }

    private double calculateFeedForward(TrapezoidProfile.State state) {
        // kS (static friction), kG (gravity), kV (velocity),
        return ElevatorConstants.kS * Math.signum(state.velocity) +
                ElevatorConstants.kG +
                ElevatorConstants.kV * state.velocity;
    }

    private void logElevatorState() {
        SmartDashboard.putNumber("Elevator Height", getHeightInches());
        SmartDashboard.putNumber("Elevator Target", targetInches);
        SmartDashboard.putString("Elevator State", targetLevel.toString());
        SmartDashboard.putNumber("Elevator Current", elevatorMotor.getOutputCurrent());
        SmartDashboard.putNumber("Elevator Velocity", nextState.velocity);
    }

    public double getHeightInches() {
        return encoder.getPosition() / ElevatorConstants.COUNTS_PER_INCH;
    }

    public boolean isAtPosition(Level position) {
        return pidController.atSetpoint() &&
                Math.abs(getHeightInches() - position.getPosition()) < 0.5;
    }

    public Level getTargetLevel() {
        return targetLevel;
    }

    public void setManualPower(double power) {
        pidController.reset();
        nextState = new TrapezoidProfile.State(getHeightInches(), 0);
        goalState = new TrapezoidProfile.State(getHeightInches(), 0);

        if (getHeightInches() >= ElevatorConstants.MAX_POSITION
                || getHeightInches() <= ElevatorConstants.MIN_POSITION) {
            power = 0;
        }

        elevatorMotor.set(MathUtil.clamp(power, ElevatorConstants.MIN_POWER, ElevatorConstants.MAX_POWER));
    }

    public void cancel() {
        pidController.reset();
        nextState = new TrapezoidProfile.State(getHeightInches(), 0);
        goalState = new TrapezoidProfile.State(getHeightInches(), 0);
        elevatorMotor.set(0);
    }
}