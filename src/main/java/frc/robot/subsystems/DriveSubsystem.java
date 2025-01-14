// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.CAN_Id_Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.Direction;
import frc.robot.utils.SwerveUtils;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  
  // Create MAXSwerveModules
  private final SwerveModule frontLeftModule = new SwerveModule(
      CAN_Id_Constants.FRONT_LEFT_DRIVE_CAN_ID,
      CAN_Id_Constants.FRONT_LEFT_STEERING_CAN_ID,
      DriveConstants.FRONT_LEFT_CHASSIS_ANGULAR_OFFSET);

  private final SwerveModule frontRightModule = new SwerveModule(
      CAN_Id_Constants.FRONT_RIGHT_DRIVE_CAN_ID,
      CAN_Id_Constants.FRONT_RIGHT_STEERING_CAN_ID,
      DriveConstants.FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET);

  private final SwerveModule backLeftModule = new SwerveModule(
      CAN_Id_Constants.BACK_LEFT_DRIVE_CAN_ID,
      CAN_Id_Constants.BACK_LEFT_STEERING_CAN_ID,
      DriveConstants.BACK_LEFT_CHASSIS_ANGULAR_OFFSET);

  private final SwerveModule backRightModule = new SwerveModule(
      CAN_Id_Constants.BACK_RIGHT_DRIVE_CAN_ID,
      CAN_Id_Constants.BACK_RIGHT_STEERING_CAN_ID,
      DriveConstants.BACK_RIGHT_CHASSIS_ANGULAR_OFFSET);

  // The gyro sensor
  private final Pigeon2 gyro = new Pigeon2(CAN_Id_Constants.PIGEON_GYRO_CAN_ID);

  // PID Controller for orientation to supplied angle
  public final PIDController orientationController;

  // Odometry class for tracking robot pose
  SwerveDriveOdometry odometry = new SwerveDriveOdometry(
      DriveConstants.DRIVE_KINEMATICS,
      Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble()),
      new SwerveModulePosition[] {
          frontLeftModule.getPosition(),
          frontRightModule.getPosition(),
          backLeftModule.getPosition(),
          backRightModule.getPosition()
      });

  public DriveSubsystem() {

    //TODO: PID Tuning could be improved
    orientationController = new PIDController(0.01, 0, 0);
    orientationController.enableContinuousInput(-180, 180);
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    odometry.update(
        Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble()),
        new SwerveModulePosition[] {
            frontLeftModule.getPosition(),
            frontRightModule.getPosition(),
            backLeftModule.getPosition(),
            backRightModule.getPosition()
        });
    SmartDashboard.putNumber("DSTargetAngle", frontLeftModule.getTargetAngle());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose (x / y coordinates and rotation)
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetPose(Pose2d pose) {
    odometry.resetPosition(
        Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble()),
        new SwerveModulePosition[] {
            frontLeftModule.getPosition(),
            frontRightModule.getPosition(),
            backLeftModule.getPosition(),
            backRightModule.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot while it adjusts to a specified orientation. 
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param direction     Direction to orient front of robot towards.
   */
  public void driveAndOrient(double xSpeed, double ySpeed, Direction direction) {
    this.driveAndOrient(xSpeed, ySpeed, SwerveUtils.normalizeAngle(SwerveUtils.directionToAngle(direction, this.getHeading())));
  }

    /**
   * Method to drive the robot while it adjusts to a specified orientation. 
   *
   * @param xSpeed            Speed of the robot in the x direction (forward).
   * @param ySpeed            Speed of the robot in the y direction (sideways).
   * @param targetHeading     Target heading (angle) robot should face
   */
  public void driveAndOrient(double xSpeed, double ySpeed, double target) {
    double currentHeading = this.getHeading();
    double targetHeading = SwerveUtils.normalizeAngle(target);
    
    // The left stick controls translation of the robot.
    // Automatically turn to face the supplied heading
    this.drive(
        xSpeed,
        ySpeed,
        this.orientationController.calculate(currentHeading, targetHeading),
        true);
  }


  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeed * DriveConstants.MAX_SPEED_METERS_PER_SECOND;
    double ySpeedDelivered = ySpeed * DriveConstants.MAX_SPEED_METERS_PER_SECOND;
    double rotDelivered = rot * DriveConstants.MAX_ANGULAR_SPEED;

    var swerveModuleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.MAX_SPEED_METERS_PER_SECOND);
    frontLeftModule.setDesiredState(swerveModuleStates[0]);
    frontRightModule.setDesiredState(swerveModuleStates[1]);
    backLeftModule.setDesiredState(swerveModuleStates[2]);
    backRightModule.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    frontLeftModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    frontRightModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    backLeftModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    backRightModule.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.MAX_SPEED_METERS_PER_SECOND);
    frontLeftModule.setDesiredState(desiredStates[0]);
    frontRightModule.setDesiredState(desiredStates[1]);
    backLeftModule.setDesiredState(desiredStates[2]);
    backRightModule.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    frontLeftModule.resetEncoders();
    backLeftModule.resetEncoders();
    frontRightModule.resetEncoders();
    backRightModule.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return SwerveUtils.normalizeAngle(gyro.getYaw().getValueAsDouble());
  }
}
