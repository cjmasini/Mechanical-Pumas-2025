package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

    private final NetworkTable limelightTable;
    private final DriveSubsystem driveSubsystem;
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
     * Returns the most recent robot pose computed from vision (via MegaTag2 pipeline).
     */
    public Pose2d getLatestRobotPose() {
        return latestRobotPose;
    }

    /**
     * Periodically update the robot's absolute pose using the MegaTag2 pipeline output,
     */
    @Override
    public void periodic() {
        // // Update absolute robot pose from MegaTag2 data.
        // double[] botposeArray = limelightTable.getEntry("botpose").getDoubleArray(new double[6]);
        // if (botposeArray.length >= 6) {
        //     double x = botposeArray[0];
        //     double y = botposeArray[1];
        //     double yawDegrees = botposeArray[5];
        //     Pose2d robotPose = new Pose2d(x, y, Rotation2d.fromDegrees(yawDegrees));
        //     latestRobotPose = robotPose;
        //     driveSubsystem.updatePose(robotPose);
        // }
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
