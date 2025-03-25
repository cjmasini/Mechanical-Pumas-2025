package frc.robot.commands;

import frc.robot.Constants.AutonConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.commands.DriveToReefCommand.ReefPosition;
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

    private static final double VISION_FORWARD_OFFSET = -.2;
    private static final double LEFT_OFFSET = -0.0;
    private static final double RIGHT_OFFSET = 0.0;

    private boolean useTOFForForward = false;

    public DriveToReefContinuousCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem,
            ReefPosition reefPosition) {
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.reefPosition = reefPosition;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        useTOFForForward = false;
        Pose2d currentPose = new Pose2d(visionSubsystem.getRobotOffset().getX(),
                visionSubsystem.getRobotOffset().getY(), Rotation2d.fromDegrees(driveSubsystem.getGyroOrientation()));

        driveSubsystem.updatePose(currentPose);

    }

    @Override
    public void execute() {
        double desiredLateralOffset = (reefPosition == ReefPosition.LEFT) ? LEFT_OFFSET : RIGHT_OFFSET;
        Pose2d desiredPose = new Pose2d(VISION_FORWARD_OFFSET, desiredLateralOffset,
                Rotation2d.fromDegrees(driveSubsystem.getGyroOrientation() + visionSubsystem.getTa()));
        // logPose("Current", visionSubsystem.getRobotOffset());
        // logPose("Desired", desiredPose);

        driveSubsystem.driveToPose(desiredPose);
        // Determine the desired lateral offset based on which side of the reef we're
        // targeting.
        Pose2d currentPose = new Pose2d(visionSubsystem.getRobotOffset().getX(),
        visionSubsystem.getRobotOffset().getY(),
        Rotation2d.fromDegrees(driveSubsystem.getGyroOrientation()));

        // logPose("Current", currentPose);
        // logPose("Desired", desiredPose);

        if (currentPose.equals(new Pose2d())) {
        driveSubsystem.driveToPose(desiredPose);
        } else {
        driveSubsystem.updatePose(new Pose2d(currentPose.getX(), currentPose.getY(),
        Rotation2d.fromDegrees(driveSubsystem.getGyroOrientation())));
        driveSubsystem.driveToOffset(desiredPose, currentPose);
        }
    }

    @Override
    public boolean isFinished() {
        double tofDistance = driveSubsystem.getTOFDistance();
        return false;// tofDistance < TOF_TARGET_DISTANCE;
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