package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ElevatorConstants.Level;
import frc.robot.commands.DriveToReefCommand.ReefPosition;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * Move elevator to a specific position
 */
public class ScoreCoralCommand extends SequentialCommandGroup {

    /**
     * Command for setting the elevator to a specific position
     *
     * @param levelChooser
     * @param elevatorSubsystem
     * @param coralSubsystem
     * @param driveSubsystem
     * @param visionSubsystem
     * @param side
     */
    public ScoreCoralCommand(SendableChooser<Level> levelChooser, ElevatorSubsystem elevatorSubsystem,
            CoralSubsystem coralSubsystem, DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem,
            ReefPosition side) {
        addRequirements(elevatorSubsystem, coralSubsystem);

        Command driveToReefCommand = new DriveToReefCommand(driveSubsystem, visionSubsystem, side);
        Command raiseElevator = new AutoSetElevatorLevelCommand(levelChooser, elevatorSubsystem);
        Command scoreCoral = new EjectCoralCommand(coralSubsystem);
        Command lowerElevator = new SetElevatorLevelCommand(Level.DOWN, elevatorSubsystem);
        this.addCommands(raiseElevator, scoreCoral, lowerElevator);
        // this.addCommands(driveToReefCommand, raiseElevator, scoreCoral, lowerElevator);

    }

    /**
     * Command for setting the elevator to a supplied position
     * 
     * @param targetLevel
     * @param elevatorSubsystem
     * @param coralSubsystem
     */
    public ScoreCoralCommand(Level targetLevel, ElevatorSubsystem elevatorSubsystem, CoralSubsystem coralSubsystem) {
        addRequirements(elevatorSubsystem, coralSubsystem);
        Command raiseElevator = new SetElevatorLevelCommand(targetLevel, elevatorSubsystem);
        Command scoreCoral = new EjectCoralCommand(coralSubsystem);
        Command lowerElevator = new SetElevatorLevelCommand(Level.DOWN, elevatorSubsystem).withTimeout(1);
        this.addCommands(raiseElevator, scoreCoral, lowerElevator);
    }

    /**
     * Command for setting the elevator to a supplied position
     * 
     * @param driveSubsystem
     * @param visionSubsystem
     * @param targetLevel
     * @param elevatorSubsystem
     * @param coralSubsystem
     * @param side
     */
    public ScoreCoralCommand(SendableChooser<Level> autoChooser, ElevatorSubsystem elevatorSubsystem,
            CoralSubsystem coralSubsystem, ReefPosition side) {
        addRequirements(elevatorSubsystem, coralSubsystem);
        Command raiseElevator = new AutoSetElevatorLevelCommand(autoChooser, elevatorSubsystem);
        Command scoreCoral = new EjectCoralCommand(coralSubsystem);
        this.addCommands(raiseElevator, scoreCoral);
    }
}
