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
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be
 * declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class CANIdConstants {
        // Drive Motor CAN Ids
        public static final int FRONT_LEFT_DRIVE_CAN_ID = 5;
        public static final int FRONT_RIGHT_DRIVE_CAN_ID = 3;
        public static final int BACK_LEFT_DRIVE_CAN_ID = 1;
        public static final int BACK_RIGHT_DRIVE_CAN_ID = 7;

        public static final int FRONT_LEFT_STEERING_CAN_ID = 6;
        public static final int FRONT_RIGHT_STEERING_CAN_ID = 4;
        public static final int BACK_LEFT_STEERING_CAN_ID = 2;
        public static final int BACK_RIGHT_STEERING_CAN_ID = 8;

        // Gyro CAN Id
        public static final int PIGEON_GYRO_CAN_ID = 21;
        public static final int INTAKE_CAN_RANGE_ID = 22;
        public static final int FORWARD_CAN_RANGE_ID = 23;

        // Elevator CAN Ids
        public static final int LEFT_ELEVATOR_CAN_ID = 11;
        public static final int RIGHT_ELEVATOR_CAN_ID = 12;

        // Motor CAN Ids
        public static final int CORAL_MOTOR_CONTROLLER_ID = 51;
        public static final int CLIMB_MOTOR_CONTROLLER_ID = 52;
        public static final int LEFT_INTAKE_WINCH_CAN_ID = 53;
        public static final int RIGHT_INTAKE_WINCH_CAN_ID = 54;
        public static final int CONVEYOR_BELT_CAN_ID = 55;
    }

    public static final class ElevatorConstants {
        public static final double COUNTS_PER_INCH = .4664;

        // PID / feedforward Constants
        // TODO: Update these - Currently set based on reca.lc estimates
        public static final double kP = 0.1;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.0;
        public static final double kV = 0.0;
        public static final double kG = 0.0;
        public static final double kA = 0.0;

        // Elevator Contraints - all in inches / seconds
        // TODO: Update these - Currently set based on reca.lc estimates
        public static final double MAX_VELOCITY = 59.14;
        public static final double MAX_ACCELERATION = 181.5;
        public static final double MAX_POSITION = 50.0;
        public static final double MIN_POSITION = 0.0;
        public static final double POSITION_TOLERANCE = 0.25;
        public static final double MIN_POWER = -1;
        public static final double MAX_POWER = 1;

        public static final double DOWN = 0;
        public static final double L1 = 11;
        public static final double L2 = 11;
        public static final double L3 = 23;
        public static final double L4 = 48.5;
        // public static final int A2 = 10;
        // public static final int A3 = 20;
        // public static final int NET = 50;

        public static enum Level {
            DOWN(ElevatorConstants.DOWN),
            L1(ElevatorConstants.L1),
            L2(ElevatorConstants.L2),
            L3(ElevatorConstants.L3),
            L4(ElevatorConstants.L4);
            // NET(ElevatorConstants.NET),
            // A2(ElevatorConstants.A2),
            // A3(ElevatorConstants.A3);

            private final double position;

            Level(double position) {
                this.position = position;
            }

            public double getPosition() {
                return position;
            }
        }
    }

    // TODO: Set raised position to real value
    public static final class IntakeConstants {
        public static final double LOWERED_POSITION = 0;
        public static final double RAISED_POSITION = 10;
    }

    public static final class RobotConstants {
        // Chassis configuration
        public static final double WHEEL_BASE = Units.inchesToMeters(29.5);
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

        public static final double GYRO_OFFSET = 0;

        // Enum for auto-orienting to field directions
        public enum Direction {
            FORWARD, BACKWARD, LEFT, RIGHT
        }
    }

    public static final class ModuleConstants {
        public static final double MAX_ANGULAR_SPEED = 2 * Math.PI; // radians per second

        // Calculations required for driving motor conversion factors and feed forward

        // The MAXSwerve module can be configured with one of three pinion gears: 12T
        // (Low),
        // 13T (Medium), or 14T (High).

        public static final int DRIVE_MOTOR_PINION_TEETH = 13;

        public static final double DRIVE_MOTOR_FREE_SPEED_RPM = 6784; // 6784 RPM for NEO Vortex
        public static final double WHEEL_CIRCUMFERENCE_IN_METERS = 0.23938936;
        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
        // teeth on the bevel pinion
        public static final double DRIVE_MOTOR_REDUCTION = (45.0 * 22) / (DRIVE_MOTOR_PINION_TEETH * 15);
        public static final double DRIVE_WHEEL_FREE_SPEED_IN_MPS = (DRIVE_MOTOR_FREE_SPEED_RPM / 60
                * WHEEL_CIRCUMFERENCE_IN_METERS)
                / DRIVE_MOTOR_REDUCTION;
    }

    public static final class OperatorConstants {
        public static final double DRIVE_DEADBAND = 0.05;
        public static final double DPAD_SPEED_REGULATOR = .25;
    }

    public static final class AutonConstants {
        // TODO: Tune these values
        public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
        public static final PIDConstants ANGLE_PID = new PIDConstants(0.01, 0, 0);

        public static final PathFollowingController AUTON_CONTROLLER = new PPHolonomicDriveController(
                TRANSLATION_PID, // new PIDConstants(5, 0.0, 0.0), // Translation PID constants
                ANGLE_PID); // new PIDConstants(5, 0.0, 0.0)); // Rotation PID constants

        // TODO: Update these values
        public static final double LIMELIGHT_HEIGHT_METERS = 0.165;
        public static final double LIMELIGHT_MOUNTING_ANGLE_DEGREES = 5.0;
        public static final double LIMELIGHT_MOUNTING_ANGLE_RADIANS = Math
                .toRadians(AutonConstants.LIMELIGHT_MOUNTING_ANGLE_DEGREES);

        public static final double REEF_APRILTAG_HEIGHT = 0.324;
    }
}
