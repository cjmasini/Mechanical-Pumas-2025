package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * DriveToLeftReefCommand - Automically drives the robot to scoring potition to
 * the left of a reef tag
 */
public class DriveToLeftReefCommand extends Command {
    private final DriveSubsystem driveSubsystem;
    private final VisionSubsystem visionSubsystem;

    // TODO: Move to constants file once finalized
    private static final double DESIRED_X_OFFSET = 1.0; // forward distance
    private static final double DESIRED_Y_OFFSET = 0.25; // lateral offset (to the left)

    public DriveToLeftReefCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
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

            driveSubsystem.driveToTagOffset(DESIRED_X_OFFSET, DESIRED_Y_OFFSET, limelightTx, limelightTy);
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
