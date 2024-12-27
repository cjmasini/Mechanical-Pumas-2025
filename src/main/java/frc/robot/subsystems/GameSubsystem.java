// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN_Id_Constants;

import swervelib.SwerveDrive;


public class GameSubsystem extends SubsystemBase
{
  /**
   * Example Motor
   */
  private final CANSparkMax exampleMotor;

  /**
   * Initialize {@link SwerveDrive} with the directory provided.
   *
   * @param directory Directory of swerve drive config files.
   */
  public GameSubsystem()
  {
    this.setName("Game Subsystem");

    this.exampleMotor = new CANSparkMax(CAN_Id_Constants.EXAMPLE_MOTOR_CONTROLLER_ID, CANSparkLowLevel.MotorType.kBrushless);
    // Restore defaults to known state before setting up motor
    this.exampleMotor.restoreFactoryDefaults();
    this.exampleMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    this.exampleMotor.setInverted(true);

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
