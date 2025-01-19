// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIdConstants;

public class GameSubsystem extends SubsystemBase
{
  /**
   * Example Motor
   */
  private final SparkMax exampleMotor;

  /**
   * Initialize {@link SwerveDrive} with the directory provided.
   *
   * @param directory Directory of swerve drive config files.
   */
  public GameSubsystem()
  {
    this.setName("Game Subsystem");

    this.exampleMotor = new SparkMax(CANIdConstants.EXAMPLE_MOTOR_CONTROLLER_ID, MotorType.kBrushless);
    SparkMaxConfig config = new SparkMaxConfig();
    config
      .inverted(true)
      .idleMode(IdleMode.kBrake);

  }

  /**
   * Set example motor speed to provided value
   * 
   * @param exampleMotorSpeed - Speed (0-1) to set Example Motor at
   */
  public void setExampleMotorSpeed(double exampleMotorSpeed){
    this.exampleMotor.set(exampleMotorSpeed);
  }

  @Override
  public void periodic()
  {
  }

  @Override
  public void simulationPeriodic()
  {
  }
}
