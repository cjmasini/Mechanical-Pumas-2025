package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveToReefCommand.ReefPosition;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.LimelightHelpers;

public class CenterThenDriveToReefCommand extends Command {

    public class AlignmentConstants {
        public static final double kPForward = 0.1;
        public static final double kPStrafe = 0.1;
        public static final double feedforward = 0.1;
        public static final double strafeTolerance = 1;
        public static final double forwardTolerance = 1;
        public static final double txTarget = 0;
        public static final double tyTarget = 0.5;
        public static final double LEFT_DEAD_RECKONING_OFFSET = -.1651;
        public static final double RIGHT_DEAD_RECKONING_OFFSET = .1651;
        public static final double FORWARD_DEAD_RECKONING_OFFSET = -.2;
    }

    private final DriveSubsystem drivetrain;
    private final ReefPosition side;
    private double strafeError = 0;
    private double forwardError = 0;
    private boolean visionAligned = false;
    private boolean zeroedPose = false;

    public CenterThenDriveToReefCommand(DriveSubsystem drivetrain, ReefPosition side) {
        this.drivetrain = drivetrain;
        this.side = side;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        this.visionAligned = false;
        this.zeroedPose = false;
    }

    @Override
    public void execute() {
        if (!visionAligned) {
            // read values form Limelight
            double tx = LimelightHelpers.getTX("");
            double ty = LimelightHelpers.getTY("");
            double ta = LimelightHelpers.getTA("");

            if (ta != 0.0) {
                strafeError = tx - AlignmentConstants.txTarget;
                forwardError = ty - AlignmentConstants.tyTarget;

                double forwardCommanded = -AlignmentConstants.kPForward * forwardError;
                double strafeCommanded = -AlignmentConstants.kPStrafe * strafeError;

                if (Math.abs(strafeError) < 1) {
                    if (strafeError < 0) {
                        strafeCommanded = strafeCommanded + AlignmentConstants.feedforward;
                    } else {
                        strafeCommanded = strafeCommanded - AlignmentConstants.feedforward;
                    }
                }

                if (Math.abs(forwardError) < 1) {
                    if (forwardError < 0) {
                        forwardCommanded = forwardCommanded + AlignmentConstants.feedforward;
                    } else {
                        forwardCommanded = forwardCommanded - AlignmentConstants.feedforward;
                    }
                }

                drivetrain.driveAndOrientToAprilTag(forwardCommanded, strafeCommanded, ta);
            }
        } else {
            if (!zeroedPose)
                drivetrain.updatePose(Pose2d.kZero);
            drivetrain.driveToPose(new Pose2d(
                    (this.side == ReefPosition.LEFT ? AlignmentConstants.LEFT_DEAD_RECKONING_OFFSET
                            : AlignmentConstants.RIGHT_DEAD_RECKONING_OFFSET),
                    AlignmentConstants.FORWARD_DEAD_RECKONING_OFFSET, Rotation2d.kZero));
        }
    }

    @Override
    public boolean isFinished() {
        return Math.abs(strafeError) < AlignmentConstants.strafeTolerance
                && Math.abs(forwardError) < AlignmentConstants.forwardTolerance;
    }
}
