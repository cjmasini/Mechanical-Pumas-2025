// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.thethriftybot.ThriftyNova;

import frc.robot.Constants.CANIdConstants;

public class CoralSubsystem extends CancelableSubsystemBase {
  /**
   * Motor to outtake coral pieces
   */
  private final ThriftyNova coralLauncher;

  /**
   * Initialize {@link SwerveDrive} with the directory provided.
   *
   * @param directory
   *          Directory of swerve drive config files.
   */
  public CoralSubsystem() {
    this.setName("Coral Subsystem");

    this.coralLauncher = new ThriftyNova(CANIdConstants.CORAL_MOTOR_CONTROLLER_ID);
  }

  /**
   * Set coral motor speed to provided value
   * 
   * @param coralMotorSpeed
   *          - Speed (0-1) to set coral motor at
   */
  public void setCoralMotorSpeed(double coralMotorSpeed) {
    this.coralLauncher.set(coralMotorSpeed);
  }

  public void cancel() {
    this.coralLauncher.set(0);
  }
}
