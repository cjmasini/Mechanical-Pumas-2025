// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import frc.robot.Configs;

/**
 * SwerveModule class - Tracks and controls state information for a swerve module
 * TODO: Uncomment and switch to spark flex code if needed
 */
public class SwerveModule {
  // private final SparkMax driveMotorController;
  private final SparkFlex driveMotorController;
  private final SparkMax steeringMotorController;

  private final RelativeEncoder driveEncoder;
  private final AbsoluteEncoder steeringEncoder;

  private final SparkClosedLoopController drivePIDController;
  private final SparkClosedLoopController steeringPIDController;

  private double chassisAngularOffset = 0;

  private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Configures a SwerveModule with a spark max motor controller and connected through bore
   * encoder
   */
  public SwerveModule(int drivingCANId, int steeringCANId, double chassisAngularOffset) {
    this.driveMotorController = new SparkFlex(drivingCANId, MotorType.kBrushless);
    this.steeringMotorController = new SparkMax(steeringCANId, MotorType.kBrushless);

    this.driveEncoder = driveMotorController.getEncoder();
    this.steeringEncoder = steeringMotorController.getAbsoluteEncoder();

    this.drivePIDController = driveMotorController.getClosedLoopController();
    this.steeringPIDController = steeringMotorController.getClosedLoopController();

    // Apply the respective configurations to the SPARKS. Reset parameters before
    // applying the configuration to bring the SPARK to a known good state. Persist
    // the settings to the SPARK to avoid losing them on a power cycle.
    this.driveMotorController.configure(Configs.MAXSwerveModule.drivingConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    this.steeringMotorController.configure(Configs.MAXSwerveModule.turningConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    this.chassisAngularOffset = chassisAngularOffset;
    this.desiredState.angle = new Rotation2d(steeringEncoder.getPosition());
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
    return new SwerveModuleState(this.driveEncoder.getVelocity(),
        new Rotation2d(this.steeringEncoder.getPosition() - chassisAngularOffset));
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
        this.driveEncoder.getPosition(),
        new Rotation2d(this.steeringEncoder.getPosition() - chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = 
      new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffset)));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    correctedDesiredState.optimize(new Rotation2d(this.steeringEncoder.getPosition()));

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    this.drivePIDController.setReference(correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
    this.steeringPIDController.setReference(correctedDesiredState.angle.getRadians(), ControlType.kPosition);
    SmartDashboard.putNumber("SWCommandedState", correctedDesiredState.angle.getRadians());
    SmartDashboard.putNumber("SWDesiredState", this.desiredState.angle.getRadians());
    SmartDashboard.putNumber("SWCurrentState", this.steeringEncoder.getPosition());


    this.desiredState = desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    this.driveEncoder.setPosition(0);
  }

  public double getTargetAngle() {
    return this.desiredState.angle.getRadians();
  }
  
  public double getCurrentAngle() {
    return this.steeringEncoder.getPosition();
  }
}