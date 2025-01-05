// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants.ModuleConstants;

/**
 * SwerveModule class - Tracks and controls state information for a swerve module
 * TODO: Uncomment and switch to spark flex code if needed
 */
public class SwerveModule {
  private final CANSparkMax driveMotorController;
  // private final CANSparkFlex driveMotorController;
  private final CANSparkMax steeringMotorController;

  private final RelativeEncoder driveEncoder;
  private final AbsoluteEncoder steeringEncoder;

  private final SparkPIDController drivePIDController;
  private final SparkPIDController steeringPIDController;

  private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Configures a SwerveModule with a spark max motor controller and connected through bore
   * encoder
   */
  public SwerveModule(int drivingCANId, int steeringCANId, double chassisAngularOffset) {
    driveMotorController = new CANSparkMax(drivingCANId, MotorType.kBrushless);
    // driveMotorController = new CANSparkFlex(drivingCANId, MotorType.kBrushless);
    steeringMotorController = new CANSparkMax(steeringCANId, MotorType.kBrushless);

    // Factory reset to get motor controllers to a known state before configuration
    driveMotorController.restoreFactoryDefaults();
    steeringMotorController.restoreFactoryDefaults();

    // Setup encoders and PID controllers for the drive and steering motor controllers
    driveEncoder = driveMotorController.getEncoder();
    steeringEncoder = steeringMotorController.getAbsoluteEncoder(Type.kDutyCycle);
    drivePIDController = driveMotorController.getPIDController();
    steeringPIDController = steeringMotorController.getPIDController();
    drivePIDController.setFeedbackDevice(driveEncoder);
    steeringPIDController.setFeedbackDevice(steeringEncoder);

    // Apply position and velocity conversion factors for the driving encoder. The
    // native units for position and velocity are rotations and RPM, respectively,
    // but we want meters and meters per second to use with WPILib's swerve APIs.
    driveEncoder.setPositionConversionFactor(ModuleConstants.DRIVE_ENCODER_POSITION_FACTOR);
    driveEncoder.setVelocityConversionFactor(ModuleConstants.DRIVE_ENCODER_VELECITY_FACTOR);

    // Apply position and velocity conversion factors for the turning encoder. We
    // want these in radians and radians per second to use with WPILib's swerve
    // APIs.
    steeringEncoder.setPositionConversionFactor(ModuleConstants.STEERING_ENCODER_POSITION_FACTOR);
    steeringEncoder.setVelocityConversionFactor(ModuleConstants.STEERING_ENCODER_VELECITY_FACTOR);

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    steeringEncoder.setInverted(ModuleConstants.TURNING_ENCODER_INVERTED);

    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.
    steeringPIDController.setPositionPIDWrappingEnabled(true);
    steeringPIDController.setPositionPIDWrappingMinInput(ModuleConstants.STEERING_ENCODER_POSITION_PID_MIN_INPUT);
    steeringPIDController.setPositionPIDWrappingMaxInput(ModuleConstants.STEERING_ENCODER_POSITION_PID_MAX_INPUT);

    // Set the PID gains for the driving motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    drivePIDController.setP(ModuleConstants.DRIVE_P);
    drivePIDController.setI(ModuleConstants.DRIVE_I);
    drivePIDController.setD(ModuleConstants.DRIVE_D);
    drivePIDController.setFF(ModuleConstants.DRIVE_FF);
    drivePIDController.setOutputRange(ModuleConstants.DRIVE_MIN_OUTPUT,
        ModuleConstants.DRIVE_MIN_INPUT);

    // Set the PID gains for the turning motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    steeringPIDController.setP(ModuleConstants.STEERING_P);
    steeringPIDController.setI(ModuleConstants.STEERING_I);
    steeringPIDController.setD(ModuleConstants.STEERING_D);
    steeringPIDController.setFF(ModuleConstants.STEERING_FF);
    steeringPIDController.setOutputRange(ModuleConstants.STEERING_MIN_OUTPUT,
        ModuleConstants.STEERING_MIN_INPUT);

    driveMotorController.setIdleMode(ModuleConstants.DRIVE_MOTOR_IDLE_MODE);
    steeringMotorController.setIdleMode(ModuleConstants.STEERING_MOTOR_IDLE_MODE);
    driveMotorController.setSmartCurrentLimit(ModuleConstants.DRIVE_MOTOR_CURRENT_LIMIT);
    steeringMotorController.setSmartCurrentLimit(ModuleConstants.STEERING_MOTOR_CURRENT_LIMIT);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    driveMotorController.burnFlash();
    steeringMotorController.burnFlash();

    desiredState.angle = new Rotation2d(steeringEncoder.getPosition());
    driveEncoder.setPosition(0);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(driveEncoder.getVelocity(),
        new Rotation2d(steeringEncoder.getPosition() - ModuleConstants.CHASSIS_ANGULAR_OFFSET));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        driveEncoder.getPosition(),
        new Rotation2d(steeringEncoder.getPosition() - ModuleConstants.CHASSIS_ANGULAR_OFFSET));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(ModuleConstants.CHASSIS_ANGULAR_OFFSET));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
        new Rotation2d(steeringEncoder.getPosition()));

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    drivePIDController.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
    steeringPIDController.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    driveEncoder.setPosition(0);
  }
}