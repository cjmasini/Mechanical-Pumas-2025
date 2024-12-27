package frc.robot.utils;

import frc.robot.Constants.DriveConstants.Direction;

public class SwerveUtils {

    /**
     * Steps a value towards a target with a specified step size.
     * @param _current The current or starting value.  Can be positive or negative.
     * @param target The target value the algorithm will step towards.  Can be positive or negative.
     * @param stepsize The maximum step size that can be taken.
     * @return The new value for {@code _current} after performing the specified step towards the specified target.
     */
    public static double StepTowards(double _current, double target, double stepsize) {
        if (Math.abs(_current - target) <= stepsize) {
            return target;
        }
        else if (target < _current) {
            return _current - stepsize;
        }
        else {
            return _current + stepsize;
        }
    }

    /**
     * Steps a value (angle) towards a target (angle) taking the shortest path with a specified step size.
     * @param current The current or starting angle (in radians).  Can lie outside the 0 to 2*PI range.
     * @param target The target angle (in radians) the algorithm will step towards.  Can lie outside the 0 to 2*PI range.
     * @param stepsize The maximum step size that can be taken (in radians).
     * @return The new angle (in radians) for {@code _current} after performing the specified step towards the specified target.
     * This value will always lie in the range 0 to 2*PI (exclusive).
     */
    public static double StepTowardsCircular(double current, double target, double stepsize) {
        current = WrapAngle(current);
        target = WrapAngle(target);

        double stepDirection = Math.signum(target - current);
        double difference = Math.abs(current - target);
        
        if (difference <= stepsize) {
            return target;
        }
        else if (difference > Math.PI) { //does the system need to wrap over eventually?
            //handle the special case where you can reach the target in one step while also wrapping
            if (current + 2*Math.PI - target < stepsize || target + 2*Math.PI - current < stepsize) {
                return target;
            }
            else {
                return WrapAngle(current - stepDirection * stepsize); //this will handle wrapping gracefully
            }

        }
        else {
            return current + stepDirection * stepsize;
        }
    }

    /**
     * Finds the (unsigned) minimum difference between two angles including calculating across 0.
     * @param angelA An angle (in radians).
     * @param angleB An angle (in radians).
     * @return The (unsigned) minimum difference between the two angles (in radians).
     */
    public static double AngleDifference(double angelA, double angleB) {
        double difference = Math.abs(angelA - angleB);
        return difference > Math.PI? (2 * Math.PI) - difference : difference;
    }

    /**
     * Wraps an angle until it lies within the range from 0 to 2*PI (exclusive).
     * @param angle The angle (in radians) to wrap.  Can be positive or negative and can lie multiple wraps outside the output range.
     * @return An angle (in radians) from 0 and 2*PI (exclusive).
     */
    public static double WrapAngle(double angle) {
        double twoPi = 2*Math.PI;

        if (angle == twoPi) { // Handle this case separately to avoid floating point errors with the floor after the division in the case below
            return 0.0;
        }
        else if (angle > twoPi) {
            return angle - twoPi*Math.floor(angle / twoPi);
        }
        else if (angle < 0.0) {
            return angle + twoPi*(Math.floor((-angle) / twoPi)+1);
        }
        else {
            return angle;
        }
    }

    /**
     * Converts an angle into a coterminal angle between -180 and 180 degrees
     * 
     * @param angle The angle to convert
     * @return A coterminal angle between -180 and 180 degrees
     */
    public static double normalizeAngle(double angle) {
    while(angle > 180) {
        angle -= 360;
    }
    while (angle < -180) {
        angle += 360;
    }
    return angle;
    }

    /**
     * Calculate the desired angle coresponding with the supplied direction
     * 
     * @param angle The angle the robot is currently facing
     * @param direction The direction the robot should face
     * @return The angle corresponding to the desired direction
     */
    public static double directionToAngle(Direction direction, double angle) {
    double desiredAngle = 0;
    switch (direction) {
        case FORWARD:
        desiredAngle = 0;
        break;
        case LEFT:
        desiredAngle = 90;
        break;
        case RIGHT:
        desiredAngle = -90;
        break;
        case BACKWARD:
        desiredAngle = angle < 0 ? -180 : 180;
        break;
    }
    return desiredAngle;
    }
}