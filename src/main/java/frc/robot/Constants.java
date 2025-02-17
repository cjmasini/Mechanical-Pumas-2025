// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  //TODO: Update CAN IDs in this class to match your robot's configuration 
  public static final class CANIdConstants {
    // Drive Motor CAN Ids
    public static final int FRONT_LEFT_DRIVE_CAN_ID = 7;
    public static final int FRONT_RIGHT_DRIVE_CAN_ID = 5;
    public static final int BACK_LEFT_DRIVE_CAN_ID = 3;
    public static final int BACK_RIGHT_DRIVE_CAN_ID = 1;

    public static final int FRONT_LEFT_STEERING_CAN_ID = 8;
    public static final int FRONT_RIGHT_STEERING_CAN_ID = 6;
    public static final int BACK_LEFT_STEERING_CAN_ID = 4;
    public static final int BACK_RIGHT_STEERING_CAN_ID = 2;

    // Gyro CAN Id
    public static final int PIGEON_GYRO_CAN_ID = 21;

    // Elevator CAN Ids
    public static final int LEFT_ELEVATOR_CAN_ID = 11;
    public static final int RIGHT_ELEVATOR_CAN_ID = 12;

    // Motor CAN Ids
    public static final int CORAL_MOTOR_CONTROLLER_ID = 51;
    public static final int CLIMB_MOTOR_CONTROLLER_ID = 52;
  }

  public static final class ElevatorConstants {

    //TODO: Update these
    public static final double COUNTS_PER_INCH = 1.0;

    // PID Constants
    // TODO: Update these
    public static final double kP = 1.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    // TODO: Update these
    public static final double kS = 0.0;
    public static final double kV = 0.0;
    public static final double kG = 0.0;

    //Elevator Contraints
    // TODO: Update these
    public static final double MAX_VELOCITY = 1.0;
    public static final double MAX_ACCELERATION = 1.0;
    public static final double MAX_POSITION = 50.0;
    public static final double MIN_POSITION = 0.0;
    public static final double POSITION_TOLERANCE = 0.25;
    public static final double MIN_POWER = -0.5;
    public static final double MAX_POWER = 0.5;


    // TODO these values are completely made up
    public static final int DOWN = 0;
    public static final int L1 = 10;
    public static final int L2 = 20;
    public static final int L3 = 30;
    public static final int L4 = 40;
    public static final int A2 = 10;
    public static final int A3 = 20;
    public static final int INTAKE = 25;
    public static final int NET = 50;

    public static enum Level {
      DOWN(ElevatorConstants.DOWN),
      INTAKE(ElevatorConstants.INTAKE),
      NET(ElevatorConstants.NET),
      L1(ElevatorConstants.L1),
      L2(ElevatorConstants.L2),
      L3(ElevatorConstants.L3),
      L4(ElevatorConstants.L4),
      A2(ElevatorConstants.A2),
      A3(ElevatorConstants.A3);

      private final int position;

      Level(int position) {
        this.position = position;
      }

      public int getPosition() {
        return position;
      }
    }
  }

  public static final class RobotConstants {
    // Chassis configuration
    //TODO: Update for your robots size
    public static final double WHEEL_BASE = Units.inchesToMeters(30);
    // Distance between centers of right and left or front and back wheels on robot
    public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(WHEEL_BASE / 2, WHEEL_BASE / 2),
        new Translation2d(WHEEL_BASE / 2, -WHEEL_BASE / 2),
        new Translation2d(-WHEEL_BASE / 2, WHEEL_BASE / 2),
        new Translation2d(-WHEEL_BASE / 2, -WHEEL_BASE / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double FRONT_LEFT_CHASSIS_ANGULAR_OFFSET = -Math.PI / 2;
    public static final double FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET = 0;
    public static final double BACK_LEFT_CHASSIS_ANGULAR_OFFSET = Math.PI;
    public static final double BACK_RIGHT_CHASSIS_ANGULAR_OFFSET = Math.PI / 2;

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds

    public static final boolean GYRO_REVERSED = false;

    // Enum for auto-orienting to field directions
    public enum Direction {
      FORWARD,
      BACKWARD,
      LEFT,
      RIGHT
    }
  }

  public static final class ModuleConstants {
    public static final double MAX_ANGULAR_SPEED = 2 * Math.PI; // radians per second

    // Calculations required for driving motor conversion factors and feed forward

    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    //TODO: Verify this is correct for your swerve setup
    public static final int DRIVE_MOTOR_PINION_TEETH = 14;

    // TODO: Update drive motor free speed if using vortex motors
    // public static final double DRIVE_MOTOR_FREE_SPEED_RPM = 5676; // 5676 RPM for NEO V1.1
    public static final double DRIVE_MOTOR_FREE_SPEED_RPM = 6784; // 6784 RPM for NEO Vortex
    public static final double WHEEL_CIRCUMFERENCE_IN_METERS = 0.23938936;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double DRIVE_MOTOR_REDUCTION = (45.0 * 22) / (DRIVE_MOTOR_PINION_TEETH * 15);
    public static final double DRIVE_WHEEL_FREE_SPEED_IN_RPS = (DRIVE_MOTOR_FREE_SPEED_RPM / 60 * WHEEL_CIRCUMFERENCE_IN_METERS)
        / DRIVE_MOTOR_REDUCTION;
  }

  public static final class OperatorConstants {
    public static final double DRIVE_DEADBAND = 0.05;
    public static final double DPAD_SPEED_REGULATOR = .25;
  }

  public static final class AutonConstants {
    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
    public static final PIDConstants ANGLE_PID   = new PIDConstants(0.4, 0, 0.01);

    public static final PathFollowingController AUTON_CONTROLLER = new PPHolonomicDriveController(
      new PIDConstants(5, 0.0, 0.0), // Translation PID constants
      new PIDConstants(5, 0.0, 0.0)); // Rotation PID constants
  }
}
