package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutonConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class DriveToReefCommand extends Command {
    private final DriveSubsystem driveSubsystem;
    private final VisionSubsystem visionSubsystem;
    private final ReefPosition reefPosition;

    // Phase 1: Use vision to approach until ~1 meter away.
    private static final double VISION_FORWARD_OFFSET = 1.0; // meters
    private static final double LEFT_OFFSET = 0.25;   // meters (left)
    private static final double RIGHT_OFFSET = 0.0;   // meters (right)

    // Phase 2: Use the CANRange TOF sensor to finalize the approach.
    private static final double TOF_TARGET_DISTANCE = 0.2; // desired final distance (meters)
    private static final double TOF_TOLERANCE = 0.05;        // acceptable error (meters)

    // Tolerance for the vision phase transition.
    private static final double POSITION_TOLERANCE = 0.1; // meters

    private enum State {
        VISION_APPROACH,
        TOF_APPROACH
    }
    private State currentState;

    // Ensure the command only starts if a valid vision target is seen.
    private boolean validStart = false;

    public enum ReefPosition {
        LEFT, RIGHT
    }

    public DriveToReefCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem,
                              ReefPosition reefPosition) {
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.reefPosition = reefPosition;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        // Only start if a valid reef tag is seen.
        if (visionSubsystem.hasTarget() && isReefTag(visionSubsystem.getTargetID())) {
            validStart = true;
            currentState = State.VISION_APPROACH;
        } else {
            validStart = false;
        }
    }

    @Override
    public void execute() {
        if (!validStart) {
            driveSubsystem.drive(0.0, 0.0, 0.0, true);
            return;
        }

        switch (currentState) {
            case VISION_APPROACH: {
                double limelightTx;
                double limelightTy;

                if (visionSubsystem.hasTarget()) {
                    limelightTx = visionSubsystem.getTx();
                    limelightTy = visionSubsystem.getTy();
                } else {
                    // Fall back to the last cached relative pose.
                    Pose2d cachedPose = visionSubsystem.getLatestReletiveRobotPose();
                    if (cachedPose.getX() == 0 && cachedPose.getY() == 0) {
                        driveSubsystem.drive(0.0, 0.0, 0.0, true);
                        return;
                    }
                    limelightTx = cachedPose.getRotation().getDegrees();
                    double currentXOffset = cachedPose.getX();
                    double mountingAngleDegrees = Math.toDegrees(AutonConstants.LIMELIGHT_MOUNTING_ANGLE_RADIANS);
                    double computedAngle = Math.toDegrees(Math.atan(
                            (AutonConstants.REEF_APRILTAG_HEIGHT - AutonConstants.LIMELIGHT_HEIGHT_METERS)
                                    / currentXOffset));
                    limelightTy = computedAngle - mountingAngleDegrees;
                }

                // Command the drivetrain using vision-based control.
                if (reefPosition == ReefPosition.LEFT) {
                    driveSubsystem.driveToTagOffset(VISION_FORWARD_OFFSET, LEFT_OFFSET, limelightTx, limelightTy);
                } else {
                    driveSubsystem.driveToTagOffset(VISION_FORWARD_OFFSET, RIGHT_OFFSET, limelightTx, limelightTy);
                }

                // Determine the current forward offset.
                double currentXOffset;
                if (visionSubsystem.hasTarget()) {
                    double tyForCalc = visionSubsystem.getTy();
                    currentXOffset = (AutonConstants.REEF_APRILTAG_HEIGHT - AutonConstants.LIMELIGHT_HEIGHT_METERS)
                            / Math.tan(AutonConstants.LIMELIGHT_MOUNTING_ANGLE_RADIANS + Math.toRadians(tyForCalc));
                } else {
                    Pose2d cachedPose = visionSubsystem.getLatestReletiveRobotPose();
                    currentXOffset = cachedPose.getX();
                }
                // Transition to TOF-based control when within tolerance of the 1-meter offset.
                if (currentXOffset <= VISION_FORWARD_OFFSET + POSITION_TOLERANCE) {
                    currentState = State.TOF_APPROACH;
                }
                break;
            }
            case TOF_APPROACH: {
                // Use the CANRange TOF sensor with the auton drive controller for the final approach.
                double tofDistance = driveSubsystem.getTOFDistance();

                // Construct the current relative pose (assume lateral and angular errors are negligible).
                Pose2d currentPose = new Pose2d(tofDistance, 0, new Rotation2d(0));

                // Desired pose is at the target distance.
                Pose2d desiredPose = new Pose2d(TOF_TARGET_DISTANCE, 0, new Rotation2d(0));

                // Create a target trajectory state.
                PathPlannerTrajectoryState targetState = new PathPlannerTrajectoryState();
                targetState.pose = desiredPose;

                // Calculate robot-relative speeds using the auton drive controller.
                ChassisSpeeds robotRelativeSpeeds =
                        AutonConstants.AUTON_CONTROLLER.calculateRobotRelativeSpeeds(currentPose, targetState);

                // Normalize the speeds as done in driveToTagOffset.
                double normalizedX = robotRelativeSpeeds.vxMetersPerSecond / ModuleConstants.DRIVE_WHEEL_FREE_SPEED_IN_MPS;
                double normalizedY = robotRelativeSpeeds.vyMetersPerSecond / ModuleConstants.DRIVE_WHEEL_FREE_SPEED_IN_MPS;
                double normalizedRot = robotRelativeSpeeds.omegaRadiansPerSecond / ModuleConstants.MAX_ANGULAR_SPEED;

                driveSubsystem.drive(normalizedX, normalizedY, normalizedRot, true);
                break;
            }
        }
    }

    @Override
    public boolean isFinished() {
        if (!validStart) {
            return true;
        }
        if (currentState == State.TOF_APPROACH) {
            double tofDistance = driveSubsystem.getTOFDistance();
            return Math.abs(tofDistance - TOF_TARGET_DISTANCE) < TOF_TOLERANCE;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.drive(0.0, 0.0, 0.0, true);
    }

    // Helper method to determine if the target ID corresponds to a reef tag.
    private boolean isReefTag(int targetID) {
        return (targetID >= 6 && targetID <= 11) || (targetID >= 17 && targetID <= 22);
    }
}
