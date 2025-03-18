// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.core.CoreCANrange;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutonConstants;
import frc.robot.Constants.CANIdConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.RobotConstants.Direction;
import frc.robot.utils.SwerveUtils;

public class DriveSubsystem extends CancelableSubsystemBase {

  // Create Swerve Modules
  private final SwerveModule frontLeftModule = new SwerveModule(
      CANIdConstants.FRONT_LEFT_DRIVE_CAN_ID,
      CANIdConstants.FRONT_LEFT_STEERING_CAN_ID,
      RobotConstants.FRONT_LEFT_CHASSIS_ANGULAR_OFFSET);

  private final SwerveModule frontRightModule = new SwerveModule(
      CANIdConstants.FRONT_RIGHT_DRIVE_CAN_ID,
      CANIdConstants.FRONT_RIGHT_STEERING_CAN_ID,
      RobotConstants.FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET);

  private final SwerveModule backLeftModule = new SwerveModule(
      CANIdConstants.BACK_LEFT_DRIVE_CAN_ID,
      CANIdConstants.BACK_LEFT_STEERING_CAN_ID,
      RobotConstants.BACK_LEFT_CHASSIS_ANGULAR_OFFSET);

  private final SwerveModule backRightModule = new SwerveModule(
      CANIdConstants.BACK_RIGHT_DRIVE_CAN_ID,
      CANIdConstants.BACK_RIGHT_STEERING_CAN_ID,
      RobotConstants.BACK_RIGHT_CHASSIS_ANGULAR_OFFSET);

  // The gyro sensor
  private final Pigeon2 gyro = new Pigeon2(CANIdConstants.PIGEON_GYRO_CAN_ID);
  // Front facing TOF sensor
  private final CoreCANrange canRangeSensor = new CoreCANrange(CANIdConstants.FORWARD_CAN_RANGE_ID);

  // PID Controller for orientation to supplied angle
  private final PIDController orientationController;

  // Odometry class for tracking robot pose
  SwerveDriveOdometry odometry = new SwerveDriveOdometry(
      RobotConstants.DRIVE_KINEMATICS,
      Rotation2d.fromDegrees(getGyroOrientation()),
      new SwerveModulePosition[] {
          frontLeftModule.getPosition(),
          frontRightModule.getPosition(),
          backLeftModule.getPosition(),
          backRightModule.getPosition()
      });

  public DriveSubsystem() {
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
      AutoBuilder.configure(
          this::getPose, // Robot pose supplier
          this::updatePose, // Method to reset odometry (will be called if your auto has a starting pose)
          this::getRobotReletiveSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
          AutonConstants.AUTON_CONTROLLER, // PID controller for autonomous driving
          config, // RobotConfig object that holds your robot's physical properties
          () -> {
            return (DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == DriverStation.Alliance.Red) ? true : false;
          }, // Supplier that returns true if the robot is on the red alliance
          this // Reference to this subsystem to set requirements
      );
    } catch (Exception e) {
      DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", e.getStackTrace());
    }

    orientationController = new PIDController(AutonConstants.ANGLE_PID.kP, AutonConstants.ANGLE_PID.kI,
        AutonConstants.ANGLE_PID.kD);
    orientationController.enableContinuousInput(-180, 180);
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    odometry.update(
        Rotation2d.fromDegrees(getGyroOrientation()),
        new SwerveModulePosition[] {
            frontLeftModule.getPosition(),
            frontRightModule.getPosition(),
            backLeftModule.getPosition(),
            backRightModule.getPosition()
        });
    SmartDashboard.putNumber("gyroOrientation", getGyroOrientation());
  }

  /**
   * Method to drive the robot while it adjusts to a specified orientation.
   *
   * @param xSpeed
   *                  Speed of the robot in the x direction (forward).
   * @param ySpeed
   *                  Speed of the robot in the y direction (sideways).
   * @param direction
   *                  Direction to orient front of robot towards.
   */
  public void driveAndOrient(double xSpeed, double ySpeed, Direction direction) {
    this.driveAndOrient(xSpeed, ySpeed,
        SwerveUtils.normalizeAngle(SwerveUtils.directionToAngle(direction, this.getHeading())));
  }

  /**
   * Method to drive the robot while it adjusts to a specified orientation.
   *
   * @param xSpeed
   *                      Speed of the robot in the x direction (forward).
   * @param ySpeed
   *                      Speed of the robot in the y direction (sideways).
   * @param targetHeading
   *                      Target heading (angle) robot should face
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
   * Drive in a robot relative direction.
   * Accepts ChassisSpeeds object for path planner integration.
   * 
   * @param robotRelativeSpeeds
   */
  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

    SwerveModuleState[] targetStates = RobotConstants.DRIVE_KINEMATICS.toSwerveModuleStates(targetSpeeds);
    setModuleStates(targetStates);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed
   *                      Speed of the robot in the x direction (forward).
   * @param ySpeed
   *                      Speed of the robot in the y direction (sideways).
   * @param rot
   *                      Angular rate of the robot.
   * @param fieldRelative
   *                      Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeed * ModuleConstants.DRIVE_WHEEL_FREE_SPEED_IN_MPS;
    double ySpeedDelivered = ySpeed * ModuleConstants.DRIVE_WHEEL_FREE_SPEED_IN_MPS;
    double rotDelivered = rot * ModuleConstants.MAX_ANGULAR_SPEED;

    var swerveModuleStates = RobotConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                Rotation2d.fromDegrees(getGyroOrientation()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, ModuleConstants.DRIVE_WHEEL_FREE_SPEED_IN_MPS);
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
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose (x / y coordinates and rotation)
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * Updates the odometry to the specified pose.
   *
   * @param pose
   *             The pose to which to set the odometry.
   */
  public void updatePose(Pose2d pose) {
    odometry.resetPosition(
        Rotation2d.fromDegrees(getGyroOrientation()),
        new SwerveModulePosition[] {
            frontLeftModule.getPosition(),
            frontRightModule.getPosition(),
            backLeftModule.getPosition(),
            backRightModule.getPosition()
        },
        pose);
  }

  /**
   * Gets current speeds of the robot in robot-relative coordinates.
   * 
   * @return
   */
  public ChassisSpeeds getRobotReletiveSpeeds() {
    return RobotConstants.DRIVE_KINEMATICS.toChassisSpeeds(getModuleStates());
  }

  /**
   * Get current states of the swerve modules
   * 
   * @return
   */
  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
        frontLeftModule.getState(),
        frontRightModule.getState(),
        backLeftModule.getState(),
        backRightModule.getState()
    };
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates
   *                      The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, ModuleConstants.DRIVE_WHEEL_FREE_SPEED_IN_MPS);
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
    return SwerveUtils.normalizeAngle(getGyroOrientation());
  }

  /**
   * Drives the robot to achieve the desired offsets from the current offsets relative to a target.
   *
   * @param desiredOffset The Pose2d representing the target position & rotation.
   * @param currentOffset The Pose2d representing the robot's current position &
   *                      rotation.
   */
  public void driveToTagOffset(Pose2d desiredOffset, Pose2d currentOffset) {
    PathPlannerTrajectoryState targetState = new PathPlannerTrajectoryState();
    targetState.pose = desiredOffset;

    ChassisSpeeds robotRelativeSpeeds = AutonConstants.AUTON_CONTROLLER.calculateRobotRelativeSpeeds(currentOffset,
        targetState);
        SmartDashboard.putNumber("Relative_r", robotRelativeSpeeds.omegaRadiansPerSecond);
        SmartDashboard.putNumber("Relative_x", robotRelativeSpeeds.omegaRadiansPerSecond);
        SmartDashboard.putNumber("Relative_y", robotRelativeSpeeds.omegaRadiansPerSecond);

    // Normalize speeds for motor control
    double normalizedX = robotRelativeSpeeds.vxMetersPerSecond / ModuleConstants.DRIVE_WHEEL_FREE_SPEED_IN_MPS;
    double normalizedY = -1*robotRelativeSpeeds.vyMetersPerSecond / ModuleConstants.DRIVE_WHEEL_FREE_SPEED_IN_MPS;
    double normalizedRot = robotRelativeSpeeds.omegaRadiansPerSecond / ModuleConstants.MAX_ANGULAR_SPEED;

    // Drive the robot toward the desired offset with rotation correction
    drive(normalizedX, normalizedY, normalizedRot, true);
  }

  public double getGyroOrientation() {
    return gyro.getYaw().getValueAsDouble() + RobotConstants.GYRO_OFFSET;
  }

  public double getTOFDistance() {
    return canRangeSensor.getDistance().getValueAsDouble();
  }
}
