package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutonConstants;

public class VisionSubsystem extends SubsystemBase {

    private final NetworkTable limelightTable;
    private final DriveSubsystem driveSubsystem;
    // Cache the last valid relative pose computed using tx and ty.
    private Pose2d latestRobotPose = new Pose2d();

    public VisionSubsystem(DriveSubsystem driveSubsystem) {
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        this.driveSubsystem = driveSubsystem;
    }

    /**
     * Returns the horizontal offset (tx) from the Limelight (in degrees).
     */
    public double getTx() {
        return limelightTable.getEntry("tx").getDouble(0.0);
    }

    /**
     * Returns the vertical offset (ty) from the Limelight (in degrees).
     */
    public double getTy() {
        return limelightTable.getEntry("ty").getDouble(0.0);
    }

    /**
     * Returns whether a valid target is detected.
     */
    public boolean hasTarget() {
        return limelightTable.getEntry("tv").getDouble(0.0) == 1.0;
    }

    /**
     * Returns the detected target's ID (assumes the tag ID is published under "tid").
     * Returns -1 if no valid ID is found.
     */
    public int getTargetID() {
        return (int) limelightTable.getEntry("tid").getDouble(-1);
    }

    /**
     * Returns the most recent relative pose computed from vision using tx and ty.
     */
    public Pose2d getLatestReletiveRobotPose() {
        return latestRobotPose;
    }

    /**
     * Periodically update the cached relative pose using tx and ty values.
     */
    @Override
    public void periodic() {
        if (hasTarget()) {
            double limelightTx = getTx();
            double limelightTy = getTy();

            // Calculate the forward distance (x) from the target using the vertical offset.
            double currentXOffset = (AutonConstants.REEF_APRILTAG_HEIGHT - AutonConstants.LIMELIGHT_HEIGHT_METERS)
                / Math.tan(AutonConstants.LIMELIGHT_MOUNTING_ANGLE_RADIANS + Math.toRadians(limelightTy));
            
            // Calculate the lateral distance (y) using the horizontal offset.
            double currentYOffset = currentXOffset * Math.tan(Math.toRadians(limelightTx));
            
            // Construct the relative pose.
            // The rotation is set from limelightTx so that 0° indicates that the limelight is directly facing the tag.
            Pose2d relativePose = new Pose2d(currentXOffset, currentYOffset, Rotation2d.fromDegrees(limelightTx));
            
            // Cache the latest valid relative pose.
            latestRobotPose = relativePose;
        }
        
        // Optionally publish whether a target is found (e.g., for debugging)
        boolean targetFound = false;
        if (hasTarget()) {
            int targetID = getTargetID();
            if ((targetID >= 6 && targetID <= 11) || (targetID >= 17 && targetID <= 22)) {
                targetFound = true;
            }
        }
        SmartDashboard.putBoolean("Target found?", targetFound);
    }
}
