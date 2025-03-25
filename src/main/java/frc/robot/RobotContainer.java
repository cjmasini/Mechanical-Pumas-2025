// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ElevatorConstants.Level;
import frc.robot.commands.AutoSetElevatorLevelCommand;
import frc.robot.commands.AutonomousCommandFactory;
import frc.robot.commands.CancelCommand;
import frc.robot.commands.CenterThenDriveToReefCommand;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.OrientToTagCommand;
import frc.robot.commands.DriveToReefCommand;
import frc.robot.commands.DriveToReefContinuousCommand;
import frc.robot.commands.FallCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.MoveCommand;
import frc.robot.commands.RaiseIntakeCommand;
import frc.robot.commands.ScoreCoralCommand;
import frc.robot.commands.SetElevatorLevelCommand;
import frc.robot.commands.DriveToReefCommand.ReefPosition;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CoralSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final DriveSubsystem drivetrain;

  private final CoralSubsystem coralSubsystem = new CoralSubsystem();

  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

  private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();

  private final ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();

  private final VisionSubsystem visionSubsystem;

  private final CommandXboxController driverXbox = new CommandXboxController(0);

  private final SendableChooser<Command> autoChooser;

  private final AutonomousCommandFactory autoFactory;

  private SendableChooser<Level> levelChooser;

  private SendableChooser<Boolean> customAutoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    ScoreCoralCommand scoreCoralCommand = new ScoreCoralCommand(Level.L4,
    elevatorSubsystem, coralSubsystem);
    NamedCommands.registerCommand("scoreRightCoral", scoreCoralCommand);

    drivetrain = new DriveSubsystem();
    autoChooser = AutoBuilder.buildAutoChooser();
    levelChooser = new SendableChooser<>();
    customAutoChooser = new SendableChooser<>();
    visionSubsystem = new VisionSubsystem();
    autoFactory = new AutonomousCommandFactory(drivetrain, visionSubsystem,
      elevatorSubsystem, coralSubsystem, intakeSubsystem);
    MoveCommand moveCommand = new MoveCommand(this.drivetrain, driverXbox);
    drivetrain.setDefaultCommand(moveCommand);

    // Add levels for the elevator to dashboard chooser
    for (Level level : Level.values()) {
      levelChooser.addOption(level.toString(), level);
    }
    levelChooser.setDefaultOption("DOWN", Level.DOWN);

    customAutoChooser.setDefaultOption("No", false);
    customAutoChooser.addOption("Yes", true);

    configureBindings();

    SmartDashboard.putData(levelChooser);
    SmartDashboard.putData(autoChooser);
    SmartDashboard.putData(customAutoChooser);
  }

  private void configureBindings() {
    // TODO: Probably remove?
    // Eject coral out of the coral mech
    RaiseIntakeCommand raiseIntakeCommand = new RaiseIntakeCommand(intakeSubsystem);
    driverXbox.y().whileTrue(Commands.startEnd(() -> coralSubsystem.setCoralMotorSpeed(.5),
        () -> coralSubsystem.setCoralMotorSpeed(0), coralSubsystem));
    driverXbox.x().whileTrue(Commands.startEnd(() -> intakeSubsystem.setBeltSpeed(.5),
        () -> intakeSubsystem.setBeltSpeed(0), coralSubsystem));
    driverXbox.b().whileTrue(Commands.startEnd(() -> coralSubsystem.setCoralMotorSpeed(-.1),
        () -> coralSubsystem.setCoralMotorSpeed(0), coralSubsystem));

    // Climb the cage while button is pressed
    ClimbCommand climbCommand = new ClimbCommand(climbSubsystem);
    // driverXbox.a().and(driverXbox.rightTrigger().negate()).whileTrue(climbCommand);

    // Slowly lower after climbing the cage
    FallCommand fallCommand = new FallCommand(climbSubsystem);
    // driverXbox.b().and(driverXbox.rightTrigger().negate()).whileTrue(fallCommand);

    // TODO: Probably remove?
    // Drive to left reef
    Command driveToReefCommand = new CenterThenDriveToReefCommand(drivetrain, ReefPosition.RIGHT).withTimeout(5);
    Command driveToReefContinuousCommand = new DriveToReefContinuousCommand(drivetrain, visionSubsystem, ReefPosition.RIGHT).withTimeout(3);
    Command driveToPose = new OrientToTagCommand(drivetrain, visionSubsystem).withTimeout(3);
    driverXbox.a().and(driverXbox.rightTrigger().negate()).onTrue(driveToReefContinuousCommand);

    // Intake coral on left trigger press
    IntakeCommand intakeCommand = new IntakeCommand(elevatorSubsystem, coralSubsystem, intakeSubsystem);
    driverXbox.leftTrigger().onTrue(intakeCommand);
    NamedCommands.registerCommand("intakeCommand", intakeCommand);

    ScoreCoralCommand scoreLeftCoralCommand = new ScoreCoralCommand(levelChooser,
        elevatorSubsystem, coralSubsystem, drivetrain, visionSubsystem, ReefPosition.LEFT);
    driverXbox.leftBumper().onTrue(scoreLeftCoralCommand);
    NamedCommands.registerCommand("scoreRightCoral", scoreLeftCoralCommand);

    // TODO: Switch to score command when ready
    // ScoreCoralCommand scoreRightCoralCommand = new
    // ScoreCoralCommand(levelChooser, elevatorSubsystem, coralSubsystem,
    // drivetrain, visionSubsystem, ReefPosition.LEFT);
    AutoSetElevatorLevelCommand scoreRightCoralCommand = new AutoSetElevatorLevelCommand(levelChooser,
        elevatorSubsystem);
    driverXbox.rightBumper().onTrue(scoreRightCoralCommand);
    NamedCommands.registerCommand("scoreRightCoral", scoreRightCoralCommand);

    // Right trigger is used to cancel other commands and as a modifier for face
    // buttons
    CancelCommand cancelCommand = new CancelCommand(
        List.of(coralSubsystem, climbSubsystem, elevatorSubsystem));
    driverXbox.rightTrigger().onTrue(new SetElevatorLevelCommand(Level.DOWN, elevatorSubsystem).withTimeout(2));

    // Manually re-zero the gyro if it gets off during competition
    // With the pigeon Gyro, we only needed to do this because of user error in
    // setup
    InstantCommand resetGyro = new InstantCommand(() -> this.drivetrain.zeroHeading());
    driverXbox.rightStick().and(driverXbox.leftStick()).onTrue(resetGyro);

    // Toggle drive mode command, currently disabled as we did not find it necessary
    // InstantCommand toggleDriveMode = new InstantCommand(() ->
    // moveCommand.toggleFieldReletive());
    // driverXbox.y().and(driverXbox.rightTrigger().negate()).onTrue(toggleDriveMode);
  }

  // Return the auto selected in smart dashboard
  public Command getAutonomousCommand() {
    if (customAutoChooser.getSelected()) {
      return autoFactory.createAutoCommand(SmartDashboard.getString("Custom Auto", "Error"));
    } else {
      return autoChooser.getSelected();
    }
  }

  public void periodic() {
    SmartDashboard.putBoolean("Valid Custom Auto?",
        AutonomousCommandFactory.isValidAuto(SmartDashboard.getString("Custom Auto", "Error")));
  }
}