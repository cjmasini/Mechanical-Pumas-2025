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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveToReefCommand extends Command {
    private final DriveSubsystem driveSubsystem;
    private final VisionSubsystem visionSubsystem;
    private final ReefPosition reefPosition;

    private static final double MAX_TOF_APPROACH_TIME = 1.0;
    private final Timer tofTimer = new Timer();

    // Limelight Approach Parameters
    private static final double VISION_FORWARD_OFFSET = 1;
    private static final double LEFT_OFFSET = -0.3048;
    private static final double RIGHT_OFFSET = 0.3048;

    // TOF Approach Parameters
    private static final double TOF_TARGET_DISTANCE = 0.0762;
    private static final double TOF_TOLERANCE = 0.01;
    private static final double POSITION_TOLERANCE = 0.01;

    private Mode mode;
    private boolean startedWithReefTag = false;

    private enum Mode {
        VISION,
        TIME_OF_FLIGHT
    }

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
        if (visionSubsystem.hasTarget() && isReefTag(visionSubsystem.getTargetID())) {
            startedWithReefTag = true;
            mode = Mode.VISION;
        } else {
            startedWithReefTag = false;
        }
    }

    @Override
    public void execute() {
        if (!startedWithReefTag) {
            driveSubsystem.drive(0.0, 0.0, 0.0, true);
            return;
        }

        SmartDashboard.putString("Approach State", mode.toString());

        switch (mode) {
            case VISION:
                visionApproach();
                break;
            case TIME_OF_FLIGHT:
                tofApproach();
                break;
        }
    }

    /**
     * Uses vision offsets to approach the reef tag
     */
    private void visionApproach() {
        if (!visionSubsystem.hasTarget()) {
            driveSubsystem.drive(0.0, 0.0, 0.0, true);
            return;
        }

        double lateralOffset = reefPosition == ReefPosition.LEFT ? LEFT_OFFSET : RIGHT_OFFSET;

        Pose2d currentOffset = visionSubsystem.getRobotOffset();
        Pose2d desiredOffset = new Pose2d(
                VISION_FORWARD_OFFSET,
                lateralOffset,
                currentOffset.getRotation());

        driveSubsystem.driveToTagOffset(desiredOffset, currentOffset);

        if (Math.abs(currentOffset.getX() - VISION_FORWARD_OFFSET) <= POSITION_TOLERANCE &&
                Math.abs(currentOffset.getY() - lateralOffset) <= POSITION_TOLERANCE) {
            mode = Mode.TIME_OF_FLIGHT;
            tofTimer.reset();
            tofTimer.start();
        }
    }

    /**
     * Handles the TOF-based final approach
     */
    private void tofApproach() {
        double tofDistance = driveSubsystem.getTOFDistance();

        if (tofTimer.get() > MAX_TOF_APPROACH_TIME) {
            DriverStation.reportWarning("Timeout: TOF approach took too long.", false);
            end(true);
            return;
        }

        Pose2d currentPose = new Pose2d(tofDistance, 0, new Rotation2d(0));
        Pose2d desiredPose = new Pose2d(TOF_TARGET_DISTANCE, 0, new Rotation2d(0));

        PathPlannerTrajectoryState targetState = new PathPlannerTrajectoryState();
        targetState.pose = desiredPose;

        ChassisSpeeds robotRelativeSpeeds = AutonConstants.AUTON_CONTROLLER.calculateRobotRelativeSpeeds(currentPose,
                targetState);

        double normalizedX = robotRelativeSpeeds.vxMetersPerSecond / ModuleConstants.DRIVE_WHEEL_FREE_SPEED_IN_MPS;
        double normalizedY = robotRelativeSpeeds.vyMetersPerSecond / ModuleConstants.DRIVE_WHEEL_FREE_SPEED_IN_MPS;
        double normalizedRotation = robotRelativeSpeeds.omegaRadiansPerSecond / ModuleConstants.MAX_ANGULAR_SPEED;

        driveSubsystem.drive(normalizedX, normalizedY, normalizedRotation, true);
    }

    @Override
    public boolean isFinished() {
        if (!startedWithReefTag)
            return true;
        if (mode == Mode.TIME_OF_FLIGHT) {
            return Math.abs(driveSubsystem.getTOFDistance() - TOF_TARGET_DISTANCE) < TOF_TOLERANCE;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.drive(0.0, 0.0, 0.0, true);
    }

    /**
     * Determines if the detected target is a valid reef tag
     */
    private boolean isReefTag(int targetID) {
        return (targetID >= 6 && targetID <= 11) || (targetID >= 17 && targetID <= 22);
    }
}
