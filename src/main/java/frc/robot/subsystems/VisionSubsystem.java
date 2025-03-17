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
    private Pose2d latestRobotPose = new Pose2d();

    public VisionSubsystem() {
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    }

    /**
     * Returns whether the Limelight has a target in view.
     * 
     * @return true/false target detected
     */
    public boolean hasTarget() {
        return limelightTable.getEntry("tv").getDouble(0.0) == 1.0;
    }

    /**
     * Returns the ID of the target detected by the Limelight.
     * 
     * @return integer target ID
     */
    public int getTargetID() {
        return (int) limelightTable.getEntry("tid").getDouble(-1);
    }

    /**
     * Computes and returns the robot's offset (X, Y) relative to the target in
     * meters.
     * 
     * @return Pose2d robot offset
     */
    public Pose2d getRobotOffset() {
        if (!hasTarget()) {
            return new Pose2d(); // Return a zeroed pose if no target is found.
        }

        double limelightTx = limelightTable.getEntry("tx").getDouble(0.0);
        double limelightTy = limelightTable.getEntry("ty").getDouble(0.0);

        // Calculate forward distance (X) from target.
        double currentXOffset = (AutonConstants.REEF_APRILTAG_HEIGHT - AutonConstants.LIMELIGHT_HEIGHT_METERS)
                / Math.tan(AutonConstants.LIMELIGHT_MOUNTING_ANGLE_RADIANS + Math.toRadians(limelightTy));

        // Calculate lateral distance (Y) from camera center.
        double currentYOffset = currentXOffset * Math.tan(Math.toRadians(limelightTx));

        // Store the latest pose and return it.
        latestRobotPose = new Pose2d(currentXOffset, currentYOffset, Rotation2d.fromDegrees(limelightTx));
        return latestRobotPose;
    }

    /**
     * Returns the most recent computed pose.
     * 
     * @return Pose2d robot pose
     */
    public Pose2d getLatestRelativeRobotPose() {
        return latestRobotPose;
    }

    /**
     * Update dashboard with vision data
     * 
     */
    @Override
    public void periodic() {
        // Update SmartDashboard with current vision data for debugging
        SmartDashboard.putBoolean("Vision/Has Target", hasTarget());
        SmartDashboard.putNumber("Vision/Target ID", getTargetID());

        Pose2d offset = getRobotOffset();
        SmartDashboard.putNumber("Vision/Offset X", offset.getX());
        SmartDashboard.putNumber("Vision/Offset Y", offset.getY());
        SmartDashboard.putNumber("Vision/Rotation", offset.getRotation().getDegrees());
    }
}
