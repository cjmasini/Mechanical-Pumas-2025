package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants.Level;
import frc.robot.commands.DriveToReefCommand.ReefPosition;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AutonomousCommandFactory {
    private final DriveSubsystem driveSubsystem;
    private final VisionSubsystem visionSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private final CoralSubsystem coralSubsystem;

    /**
     * Constructor for AutoCommandFactory.
     *
     * @param driveSubsystem  The robot's DriveSubsystem.
     * @param visionSubsystem The robot's VisionSubsystem.
     */
    public AutonomousCommandFactory(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem,
            ElevatorSubsystem elevatorSubsystem, CoralSubsystem coralSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.coralSubsystem = coralSubsystem;
    }

    /**
     * Creates an autonomous command from a sequence string.
     * 
     * @param sequenceString String in the format LA1L (Start/End at Left coral
     *                       station, Reef
     *                       Branch A, Level 1, ) or RF4 (Right coral station, Reef
     *                       Branch F, Level 4).
     * 
     * @return The command that executes the autonomous sequence.
     */
    public Command createAutoCommand(String sequenceString) {
        // String sequenceString = SmartDashboard.getString("Test", "ERROR");
        SequentialCommandGroup autonomousSequence = new SequentialCommandGroup();
        String[] routines = sequenceString.split(",");

        for (String routineString : routines) {
            routineString = routineString.trim();
            if (routineString.length() != 3) {
                DriverStation.reportError("Invalid sequence string: " + sequenceString, false);
                continue;
            }
            char coralStation = Character.toUpperCase(routineString.charAt(0));
            Level reefLevel = mapLevelStringToReefLevel(routineString.charAt(1));
            char reefLetter = Character.toUpperCase(routineString.charAt(2));
            String pathSegment = mapReefLetterToPathSegment(reefLetter);

            String coralToReefPathPositionName;
            String reefToCoralStationPathName;
            Command driveToReefCommand;
            if (coralStation == 'L') {
                coralToReefPathPositionName = "Left " + pathSegment + " Position";
                reefToCoralStationPathName = reefLetter + " to Left";
                driveToReefCommand = new DriveToReefCommand(driveSubsystem, visionSubsystem, ReefPosition.LEFT);
            } else {
                coralToReefPathPositionName = "Right " + pathSegment + " Position";
                reefToCoralStationPathName = reefLetter + " to Right";
                driveToReefCommand = new DriveToReefCommand(driveSubsystem, visionSubsystem, ReefPosition.RIGHT);
            }

            Command coralStationToReefPositionCommand;
            Command reefToCoralStationCommand;
            try {
                PathPlannerPath coralToReefPositionPath = PathPlannerPath.fromPathFile(coralToReefPathPositionName);
                coralStationToReefPositionCommand = AutoBuilder.followPath(coralToReefPositionPath);
                PathPlannerPath reefToCoralStationPath = PathPlannerPath.fromPathFile(reefToCoralStationPathName);
                reefToCoralStationCommand = AutoBuilder.followPath(reefToCoralStationPath);
            } catch (Exception e) {
                DriverStation.reportError("Failed to load path file: " + e.getMessage(), false);
                continue;
            }

            Command scoreCoralCommand = new ScoreCoralCommand(reefLevel, elevatorSubsystem, coralSubsystem);

            // Add the commands for this instruction to the sequence.
            autonomousSequence.addCommands(
                    coralStationToReefPositionCommand,
                    driveToReefCommand,
                    scoreCoralCommand,
                    reefToCoralStationCommand);
        }

        return autonomousSequence;
    }

    /**
     * Converts the reef letter to the corresponding positioning path segment.
     *
     * @param reefLetter The letter representing the reef section.
     * @return The corresponding path segment string.
     */
    private static String mapReefLetterToPathSegment(char reefLetter) {
        switch (reefLetter) {
            case 'A':
            case 'B':
                return "AB";
            case 'C':
            case 'D':
                return "CD";
            case 'E':
            case 'F':
                return "EF";
            case 'G':
            case 'H':
                return "GH";
            case 'I':
            case 'J':
                return "IJ";
            case 'K':
            case 'L':
                return "KL";
            default:
                DriverStation.reportError("Invalid reef letter: " + reefLetter, null);
                return "";
        }
    }

    /**
     * Converts the reef letter to the corresponding positioning path segment.
     *
     * @param reefLevel The letter representing the reef section.
     * @return The corresponding path segment string.
     */
    private static Level mapLevelStringToReefLevel(char reefLevel) {
        switch (reefLevel) {
            case '1':
                return Level.L1;
            case '2':
                return Level.L2;
            case '3':
                return Level.L3;
            case '4':
                return Level.L4;
            default:
                DriverStation.reportError("Invalid reef level: " + reefLevel, null);
                return Level.DOWN;
        }
    }

}
