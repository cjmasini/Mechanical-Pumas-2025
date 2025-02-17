// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.CANIdConstants;

public class ClimbSubsystem extends CancelableSubsystemBase
{
    /**
     * Motor (with 100 to 1 gearbox) to climb
     */
    private final SparkMax climber;


    private Timer stalledTimer;

    /**
     * Initialize {@link SwerveDrive} with the directory provided.
     *
     * @param directory Directory of swerve drive config files.
     */
    public ClimbSubsystem()
    {
    this.setName("Climb Subsystem");

    this.climber = new SparkMax(CANIdConstants.CLIMB_MOTOR_CONTROLLER_ID, MotorType.kBrushless);
    SparkMaxConfig config = new SparkMaxConfig();
    config
        .inverted(true)
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(30);

    }

    /**
     * Set climb motor speed to provided value
     * 
     * @param climbMotorSpeed - Speed (0-1) to set climb motor at
     */
    public void setClimbMotorSpeed(double climbMotorSpeed){
    this.climber.set(climbMotorSpeed);
    }

    /**
     * Check if motor is stalled to avoid damaging gearbox 
     */
    @Override
    public void periodic()
    {
        // TODO: Fine tune stall current value based on data
        if (this.climber.getOutputCurrent() > 40 && !this.stalledTimer.isRunning()){
            this.stalledTimer.start();
        } else if (this.climber.getOutputCurrent() < 30 && this.stalledTimer.get() > 2){
            this.climber.set(0);
            this.stalledTimer.stop();
        }
    }

    public void cancel()
    {
        this.climber.set(0);
    }
}
