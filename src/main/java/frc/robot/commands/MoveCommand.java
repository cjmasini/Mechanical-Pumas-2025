package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.RobotConstants.Direction;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Command for note intake
 */
public class MoveCommand extends Command 
{
  private CommandXboxController driverXbox;
  private DriveSubsystem driveSubsystem;
  private boolean fieldRelative = true;

  /**
   * Command for moving the robot using the swerve drive modules and an x-box controller
   * D-PAD controls the robot at 20% speed in robot-oriented mode
   * Left Joystick controls the robot at variable speeds (0-100%) in field-oriented mode 
   *      - Cubes driver input for greater control at low speeds
   * Right Joystick controls the robot orientation / heading (direction it is facing)
   * Right Trigger + A/B/X/Y -> Robot automically orients to the corresponding field-oriented heading
   *      - Works while moving and continues to orient until button is released
   * @param drivetrain  The drive subsystem.
   * @param driverXbox The xbox controller for the robot
   */
  public MoveCommand(DriveSubsystem drivetrain, CommandXboxController driverXbox)
  {
      this.driveSubsystem = drivetrain;
      this.driverXbox = driverXbox;
      addRequirements(this.driveSubsystem);
  }

  @Override
  public void execute() {
    // Trigger mappings for fine-tuned robot-oriented adjustments using the d-pad
    if (driverXbox.povLeft().getAsBoolean()) {
      driveSubsystem.drive(0, .2, -MathUtil.applyDeadband(driverXbox.getRightX(), OperatorConstants.DRIVE_DEADBAND), false);
    } else if (driverXbox.povRight().getAsBoolean()) {
      driveSubsystem.drive(0, -.2, -MathUtil.applyDeadband(driverXbox.getRightX(), OperatorConstants.DRIVE_DEADBAND), false);
    } else if (driverXbox.povUp().getAsBoolean()) {
      driveSubsystem.drive(.2, 0, -MathUtil.applyDeadband(driverXbox.getRightX(), OperatorConstants.DRIVE_DEADBAND), false);
    } else if (driverXbox.povDown().getAsBoolean()) {
      driveSubsystem.drive(-0.2, 0, -MathUtil.applyDeadband(driverXbox.getRightX(), OperatorConstants.DRIVE_DEADBAND), false);
    } else if (driverXbox.povUpLeft().getAsBoolean()) {
      driveSubsystem.drive(.14, .14, -MathUtil.applyDeadband(driverXbox.getRightX(), OperatorConstants.DRIVE_DEADBAND), false);
    } else if (driverXbox.povUpRight().getAsBoolean()) {
      driveSubsystem.drive(.14, -.14, -MathUtil.applyDeadband(driverXbox.getRightX(), OperatorConstants.DRIVE_DEADBAND), false);
    } else if (driverXbox.povDownLeft().getAsBoolean()) {
      driveSubsystem.drive(-.14, .14, -MathUtil.applyDeadband(driverXbox.getRightX(), OperatorConstants.DRIVE_DEADBAND), false);
    } else if (driverXbox.povDownRight().getAsBoolean()) {
      driveSubsystem.drive(-.14, -.14, -MathUtil.applyDeadband(driverXbox.getRightX(), OperatorConstants.DRIVE_DEADBAND), false);
    } else { // Joystick / field-oriented based movement
      double yMovement = driverXbox.getLeftY();
      double xMovement = driverXbox.getLeftX();

      // Trigger mappings for field oriented driving while automically orienting to a supplied direction
      if(driverXbox.rightTrigger().getAsBoolean() && driverXbox.x().getAsBoolean()) {
        this.driveSubsystem.driveAndOrient(
            -MathUtil.applyDeadband(OperatorConstants.DPAD_SPEED_REGULATOR * Math.pow(yMovement, 3), OperatorConstants.DRIVE_DEADBAND),
            -MathUtil.applyDeadband(OperatorConstants.DPAD_SPEED_REGULATOR * Math.pow(xMovement, 3), OperatorConstants.DRIVE_DEADBAND),
            Direction.RIGHT);
      } else if(driverXbox.rightTrigger().getAsBoolean() && driverXbox.y().getAsBoolean()) {
        this.driveSubsystem.driveAndOrient(
            -MathUtil.applyDeadband(OperatorConstants.DPAD_SPEED_REGULATOR * Math.pow(yMovement, 3), OperatorConstants.DRIVE_DEADBAND),
            -MathUtil.applyDeadband(OperatorConstants.DPAD_SPEED_REGULATOR * Math.pow(xMovement, 3), OperatorConstants.DRIVE_DEADBAND),
            Direction.BACKWARD);
      } else if(driverXbox.rightTrigger().getAsBoolean() && driverXbox.b().getAsBoolean()) {
        this.driveSubsystem.driveAndOrient(
            -MathUtil.applyDeadband(OperatorConstants.DPAD_SPEED_REGULATOR * Math.pow(yMovement, 3), OperatorConstants.DRIVE_DEADBAND),
            -MathUtil.applyDeadband(OperatorConstants.DPAD_SPEED_REGULATOR * Math.pow(xMovement, 3), OperatorConstants.DRIVE_DEADBAND),
            Direction.LEFT);
      } else if(driverXbox.rightTrigger().getAsBoolean() && driverXbox.a().getAsBoolean()) {
        this.driveSubsystem.driveAndOrient(
            -MathUtil.applyDeadband(OperatorConstants.DPAD_SPEED_REGULATOR * Math.pow(yMovement, 3), OperatorConstants.DRIVE_DEADBAND),
            -MathUtil.applyDeadband(OperatorConstants.DPAD_SPEED_REGULATOR * Math.pow(xMovement, 3), OperatorConstants.DRIVE_DEADBAND),
            Direction.FORWARD);
      } else {
        // Default joystick controlled swerve
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        this.driveSubsystem.drive(
            -MathUtil.applyDeadband(OperatorConstants.DPAD_SPEED_REGULATOR * Math.pow(yMovement, 3), OperatorConstants.DRIVE_DEADBAND),
            -MathUtil.applyDeadband(OperatorConstants.DPAD_SPEED_REGULATOR * Math.pow(xMovement, 3), OperatorConstants.DRIVE_DEADBAND),
            -MathUtil.applyDeadband(driverXbox.getRightX(), OperatorConstants.DRIVE_DEADBAND),
            fieldRelative);
      }
    }
  }
  
  @Override
  public boolean isFinished()
  {
    return false;
  }

  /*
   * getFieldReletive - returns the robot drive mode (field or robot oriented)
   */
  public boolean getFieldReletive() {
    return this.fieldRelative;
  }

  /*
   * toggleFieldReletive - toggles the drive mode of the robot
   * 
   * Can be used to dynamically switch between robot-oriented and field-oriented modes
   */
  public void toggleFieldReletive() {
    this.fieldRelative = !this.fieldRelative;
  }

    /*
   * setFieldReletive - sets the drive mode of the robot
   * 
   * Can be used to dynamically set field-oriented or robot oriented drive modes
   */
  public void setFieldReletive(boolean fieldRelative) {
    this.fieldRelative = fieldRelative;
  }
}
