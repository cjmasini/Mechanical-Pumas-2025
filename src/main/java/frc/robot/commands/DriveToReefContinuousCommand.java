package frc.robot.commands;

import frc.robot.Constants.AutonConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveToReefContinuousCommand extends Command {
    private final DriveSubsystem driveSubsystem;
    private final VisionSubsystem visionSubsystem;
    private final ReefPosition reefPosition;

    private static final double VISION_FORWARD_OFFSET = -.025;
    private static final double LEFT_OFFSET = -0.28;
    private static final double RIGHT_OFFSET = 0.28;
    private static final double POSITION_TOLERANCE = 0.05;
    private static final double ORIENTATION_THRESHOLD_DEGREES = 1.0;
    private static final double TOF_TARGET_DISTANCE = 0.025;

    private boolean useTOFForForward = false;

    public enum ReefPosition {
        LEFT, RIGHT
    }

    public DriveToReefContinuousCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, ReefPosition reefPosition) {
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.reefPosition = reefPosition;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        useTOFForForward = false;
    }

    @Override
    public void execute() {
        // Determine the desired lateral offset based on which side of the reef we're targeting.
        double desiredLateralOffset = (reefPosition == ReefPosition.LEFT) ? LEFT_OFFSET : RIGHT_OFFSET;
        Pose2d desiredPose;
        Pose2d currentPose;

        if (!useTOFForForward) {
            if (!visionSubsystem.hasTarget()) {
                driveSubsystem.drive(0.0, 0.0, 0.0, true);
                SmartDashboard.putString("DriveMode", "No Vision");
                return;
            }
            Pose2d visionPose = visionSubsystem.getRobotOffset();
            currentPose = new Pose2d(visionPose.getX(), visionPose.getY(), Rotation2d.fromDegrees(0));
            desiredPose = new Pose2d(VISION_FORWARD_OFFSET, desiredLateralOffset, Rotation2d.fromDegrees(0));

            double orientationError = Math.abs(visionPose.getRotation().getDegrees());
            SmartDashboard.putNumber("OrientationError", orientationError);
            SmartDashboard.putString("DriveMode", "Vision");

            // Once oriented to the reef, switch forward offset to TOF sensor
            if (orientationError < ORIENTATION_THRESHOLD_DEGREES) {
                double tofDistance = driveSubsystem.getTOFDistance();
                Pose2d resetPose = new Pose2d(tofDistance, visionPose.getY(), Rotation2d.fromDegrees(0));
                driveSubsystem.updatePose(resetPose);
                useTOFForForward = true;
                SmartDashboard.putString("DriveMode", "Switching to TOF");
            }
        } else {
            double forwardMeasurement = -1.0 * driveSubsystem.getTOFDistance();
            double lateralMeasurement;
            if (visionSubsystem.hasTarget()) {
                lateralMeasurement = visionSubsystem.getRobotOffset().getY();
                SmartDashboard.putString("LateralSource", "Vision");
            } else {
                // If vision is lost, fall back on the drive subsystem's odometry
                lateralMeasurement = driveSubsystem.getPose().getY();
                SmartDashboard.putString("LateralSource", "Odometry");
            }
            currentPose = new Pose2d(forwardMeasurement, lateralMeasurement, Rotation2d.fromDegrees(0));
            desiredPose = new Pose2d(TOF_TARGET_DISTANCE, desiredLateralOffset, Rotation2d.fromDegrees(0));
            SmartDashboard.putString("DriveMode", "TOF Forward");
        }

        PathPlannerTrajectoryState targetState = new PathPlannerTrajectoryState();
        targetState.pose = desiredPose;
        ChassisSpeeds speeds = AutonConstants.AUTON_CONTROLLER.calculateRobotRelativeSpeeds(currentPose, targetState);

        double normalizedX = -speeds.vxMetersPerSecond / ModuleConstants.DRIVE_WHEEL_FREE_SPEED_IN_MPS;
        double normalizedY = speeds.vyMetersPerSecond / ModuleConstants.DRIVE_WHEEL_FREE_SPEED_IN_MPS;
        double normalizedRot = speeds.omegaRadiansPerSecond / ModuleConstants.MAX_ANGULAR_SPEED;

        driveSubsystem.drive(normalizedX, normalizedY, normalizedRot, true);
        logPose("CurrentPose", currentPose);
        logPose("DesiredPose", desiredPose);
    }

    @Override
    public boolean isFinished() {
        if (useTOFForForward) {
            double tofDistance = driveSubsystem.getTOFDistance();
            return Math.abs(tofDistance - TOF_TARGET_DISTANCE) < POSITION_TOLERANCE;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.drive(0.0, 0.0, 0.0, true);
    }

    private void logPose(String name, Pose2d pose) {
        SmartDashboard.putNumber(name + "_x", pose.getX());
        SmartDashboard.putNumber(name + "_y", pose.getY());
        SmartDashboard.putNumber(name + "_rot", pose.getRotation().getDegrees());
    }
}
