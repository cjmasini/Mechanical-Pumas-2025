package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * DriveToLeftReefCommand - Automically drives the robot to scoring potition to
 * the left of a reef tag
 */
public class DriveToReefCommand extends Command {
    private final DriveSubsystem driveSubsystem;
    private final VisionSubsystem visionSubsystem;
    private final ReefPosition reefPosition;

    // TODO: Move to constants file once finalized
    private static final double FORWARD_OFFSET = 1.0; // forward distance
    private static final double LEFT_OFFSET = 0.25; // left lateral offset
    private static final double RIGHT_OFFSET = 0.0; // right lateral offset

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
    public void execute() {
        // First, ensure a target is detected.
        if (!visionSubsystem.hasTarget()) {
            driveSubsystem.drive(0.0, 0.0, 0.0, true);
            return;
        }

        int targetID = visionSubsystem.getTargetID();

        if ((targetID >= 6 && targetID <= 11) || (targetID >= 17 && targetID <= 22)) {
            double limelightTx = visionSubsystem.getTx();
            double limelightTy = visionSubsystem.getTy();

            if (reefPosition == ReefPosition.LEFT) {
                driveSubsystem.driveToTagOffset(FORWARD_OFFSET, LEFT_OFFSET, limelightTx, limelightTy);
            } else {
                driveSubsystem.driveToTagOffset(FORWARD_OFFSET, RIGHT_OFFSET, limelightTx, limelightTy);
            }
        } else {
            driveSubsystem.drive(0.0, 0.0, 0.0, true);
            return;
        }
    }

    // TODO: Figure out how to define end behavior
    @Override
    public boolean isFinished() {
        return false;
    }
}
