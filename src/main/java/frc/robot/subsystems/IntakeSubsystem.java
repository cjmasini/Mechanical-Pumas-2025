package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.core.CoreCANrange;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.thethriftybot.ThriftyNova;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.CANIdConstants;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends CancelableSubsystemBase {
    // private final SparkMax winchMotor;
    // private final SparkMax followerMotor;
    // private final RelativeEncoder encoder;
    private final ThriftyNova conveyorBeltMotor;
    private final CoreCANrange canRangeSensor; 

    public IntakeSubsystem() {
        // winchMotor = new SparkMax(CANIdConstants.LEFT_INTAKE_WINCH_CAN_ID, MotorType.kBrushless);
        // SparkMaxConfig winchMotorConfig = new SparkMaxConfig();
        // winchMotorConfig.idleMode(IdleMode.kBrake);
        // winchMotorConfig.smartCurrentLimit(20);
        // winchMotorConfig.voltageCompensation(12.0);
        // winchMotor.configure(winchMotorConfig, ResetMode.kResetSafeParameters, null);

        // followerMotor = new SparkMax(CANIdConstants.RIGHT_INTAKE_WINCH_CAN_ID, MotorType.kBrushless);
        // SparkMaxConfig followerConfig = new SparkMaxConfig();
        // followerConfig.follow(winchMotor, true);
        // followerMotor.configure(followerConfig, null, null);

        conveyorBeltMotor = new ThriftyNova(CANIdConstants.CONVEYOR_BELT_CAN_ID);

        // encoder = winchMotor.getEncoder();

        canRangeSensor = new CoreCANrange(CANIdConstants.INTAKE_CAN_RANGE_ID); 
    }

    // /**
    //  * Gets the current position of the intake drawbridge
    //  * 
    //  * @return The current position of the intake drawbridge
    //  */
    // public double getIntakePosition() {
    //     return encoder.getPosition();
    // }

    // /**
    //  * Sets the intake winch speed
    //  */
    // public void setWinchSpeed(double speed) {
    //     speed = MathUtil.clamp(speed, -1, 1);
    //     winchMotor.set(speed);
    // }

    // /**
    //  * Checks if the intake is lowered
    //  * 
    //  * @return True if the intake is lowered
    //  */
    // public boolean isLowered() {
    //     return encoder.getPosition() <= IntakeConstants.LOWERED_POSITION;
    // }

    // /**
    //  * Checks if the intake is raised
    //  * 
    //  * @return True if the intake is raised
    //  */
    // public boolean isRaised() {
    //     return encoder.getPosition() >= IntakeConstants.RAISED_POSITION;
    // }

    /**
     * Sets the conveyor belt speed
     * @param speed The speed to set the conveyor belt to
     */
    public void setBeltSpeed(double speed) {
        speed = MathUtil.clamp(speed, -1, 1);
        System.out.println("Speed" + speed);
        conveyorBeltMotor.set(speed);
    }

    /**
     * Gets the distance from the CAN range sensor in inches
     * @return The distance from the CAN range sensor in inches
     */
    public double getTOFDistanceInches() {
        return canRangeSensor.getDistance().getValueAsDouble()*39.3701;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Range", getTOFDistanceInches());
    }

    @Override
    public void cancel() {
        this.setBeltSpeed(0);
    }
}