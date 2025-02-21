package frc.robot.utils;

import static java.util.Map.entry;
import java.util.Map;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * FieldConstants contains the absolute field locations of the AprilTags for the 2025 game.
 * Data based on the published AprilTag map JSON:
 * {
 *   "fieldlength":17.5482504,
 *   "fieldwidth":8.0519016,
 *   "fiducials":[
 *     { "id": 1, "transform": [ m00, m01, m02, x, m10, m11, m12, y, ... ] },
 *     ... 
 *   ],
 *   "type":"frc"
 * }
 *
 * For each fiducial, x = transform[3] and y = transform[7], and the heading is computed as:
 *   theta = atan2(transform[1], transform[0])
 */
public final class FieldConstants {
  public static final Map<Integer, Pose2d> APRILTAG_FIELD_POSITIONS = Map.ofEntries(
      entry(1,  new Pose2d(7.923198000000001, -3.37068,  new Rotation2d(Math.atan2(-0.8090169943749473, -0.5877852522924729)))),
      entry(2,  new Pose2d(7.923198000000001,  3.37048,   new Rotation2d(Math.atan2( 0.8090169943749473, -0.5877852522924734)))),
      entry(3,  new Pose2d(2.78681,           4.02961,   new Rotation2d(Math.atan2(1, -2.220446049250313e-16)))),
      entry(4,  new Pose2d(0.50208,           2.111656,  new Rotation2d(0))),
      entry(5,  new Pose2d(0.50208,          -2.111094,  new Rotation2d(0))),
      entry(6,  new Pose2d(4.700446,         -0.719682,  new Rotation2d(Math.atan2(0.8660254037844386,  0.5000000000000001)))),
      entry(7,  new Pose2d(5.116498,         -0.0001,    new Rotation2d(0))),
      entry(8,  new Pose2d(4.700446,          0.719482,  new Rotation2d(Math.atan2(-0.8660254037844386, 0.5000000000000001)))),
      entry(9,  new Pose2d(3.869358,          0.719482,  new Rotation2d(Math.atan2(-0.8660254037844388, -0.5)))),
      entry(10, new Pose2d(3.453306,         -0.0001,    new Rotation2d(Math.atan2(-1.2246467991473532e-16, -1)))),
      entry(11, new Pose2d(3.869358,         -0.719682,  new Rotation2d(Math.atan2(0.8660254037844384, -0.5)))),
      entry(12, new Pose2d(-7.922846,        -3.37068,   new Rotation2d(Math.atan2(-0.8090169943749473,  0.5877852522924731)))),
      entry(13, new Pose2d(-7.922846,         3.37048,   new Rotation2d(Math.atan2( 0.8090169943749475,  0.587785252292473)))),
      entry(14, new Pose2d(-0.501728,         2.111656,  new Rotation2d(Math.atan2(-1.2246467991473532e-16, -0.8660254037844388)))),
      entry(15, new Pose2d(-0.501728,        -2.111094,  new Rotation2d(Math.atan2(-1.2246467991473532e-16, -0.8660254037844388)))),
      entry(16, new Pose2d(-2.786458,        -4.02981,   new Rotation2d(Math.atan2(-1.0000000000000002, -2.220446049250313e-16)))),
      entry(17, new Pose2d(-4.700094,        -0.719682,  new Rotation2d(Math.atan2(0.8660254037844384, -0.5000000000000002)))),
      entry(18, new Pose2d(-5.1164,          -0.0001,    new Rotation2d(Math.atan2(-1.2246467991473532e-16, -1)))),
      entry(19, new Pose2d(-4.700094,         0.719482,  new Rotation2d(Math.atan2(-0.8660254037844388, -0.5)))),
      entry(20, new Pose2d(-3.86926,          0.719482,  new Rotation2d(Math.atan2(-0.8660254037844386,  0.5000000000000001)))),
      entry(21, new Pose2d(-3.452954,        -0.0001,    new Rotation2d(0))),
      entry(22, new Pose2d(-3.86926,         -0.719682,  new Rotation2d(Math.atan2(0.8660254037844386,  0.5000000000000001)))
      )
  );

  private FieldConstants() {
    // Prevent instantiation.
  }
}
