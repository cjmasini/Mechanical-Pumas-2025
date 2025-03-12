package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorConstants.Level;
import frc.robot.commands.DriveToReefCommand.ReefPosition;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AutonomousCommandFactory {
    private final DriveSubsystem driveSubsystem;
    private final VisionSubsystem visionSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private final CoralSubsystem coralSubsystem;
    private final IntakeSubsystem intakeSubsystem;

    /**
     * Constructor for AutoCommandFactory.
     *
     * @param driveSubsystem  The robot's DriveSubsystem.
     * @param visionSubsystem The robot's VisionSubsystem.
     */
    public AutonomousCommandFactory(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem,
            ElevatorSubsystem elevatorSubsystem, CoralSubsystem coralSubsystem, IntakeSubsystem intakeSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.coralSubsystem = coralSubsystem;
        this.intakeSubsystem = intakeSubsystem;
    }

    /**
     * Creates an autonomous command from a sequence string.
     * 
     * @param sequenceString String in the format r4L:A4L,B3R
     *                       First Character:
     *                       * Starting string has r/c/l as the options for robot
     *                       starting postion
     *                       * Other strings have A-L as the options for reef letter
     *                       Second Character:
     *                       * Seconds character has 1-4 as the options for reef
     *                       level
     *                       Third Character:
     *                       * Third character has L/R as the options for coral
     *                       station to return to
     * 
     * 
     * @return The command that executes the autonomous sequence.
     */
    public Command createAutoCommand(String sequenceString) {
        SequentialCommandGroup autonomousSequence = new SequentialCommandGroup();
        String[] startAndBody = sequenceString.split(":");
        String start = startAndBody[0];
        String[] routines;
        if (startAndBody.length == 1) {
            routines = new String[0];
        } else {
            routines = startAndBody[1].split(",");
        }

        // Parse the starting command instructions
        String startToReefPathName = "";
        String startingReefToCoralStationPathName = "";
        ReefPosition startingReefPosition = ReefPosition.RIGHT;
        char previousCoralStation = 'S';
        start = start.trim();
        if (start.length() != 3) {
            DriverStation.reportError("Invalid starting command: " + start, false);
            return new InstantCommand(); // return null command if starting sequence doesn't process as expected
        }
        char startingPosition = Character.toLowerCase(start.charAt(0));
        char startingLevel = start.charAt(1);
        char startingCoralStation = Character.toUpperCase(start.charAt(2));
        previousCoralStation = startingCoralStation;
        // Convert starting reef level character to Level enum.
        Level startingReefLevel = mapLevelStringToReefLevel(startingLevel);

        // Determine the starting path names and reef position using helper functions.
        startToReefPathName = getStartToReefPathName(startingPosition);
        if (startToReefPathName.isEmpty()) {
            DriverStation.reportError("Invalid starting position: " + startingPosition, false);
            return new InstantCommand(); // return null command if starting sequence doesn't process as expected
        }
        startingReefToCoralStationPathName = getStartingReefToCoralStationPathName(startingPosition,
                startingCoralStation);
        startingReefPosition = startingCoralStation == 'R' ? ReefPosition.RIGHT : ReefPosition.LEFT;
        try {
            PathPlannerPath startToReefPath = PathPlannerPath.fromPathFile(startToReefPathName);
            Command startToReefPositionCommand = AutoBuilder.followPath(startToReefPath);
            Command scoreCoralCommand = new ScoreCoralCommand(startingReefLevel, driveSubsystem, visionSubsystem, elevatorSubsystem, coralSubsystem,
                    startingReefPosition);
            PathPlannerPath startingReefToCoralStationPath = PathPlannerPath
                    .fromPathFile(startingReefToCoralStationPathName);
            Command startingReefToCoralStationCommand = AutoBuilder.followPath(startingReefToCoralStationPath);
            autonomousSequence.addCommands(startToReefPositionCommand, scoreCoralCommand,
                    startingReefToCoralStationCommand);
        } catch (Exception e) {
            DriverStation.reportError("Failed to load path file: " + e.getMessage(), false);
            return new InstantCommand();
        }

        // Parse remaining command instructions
        for (String routineString : routines) {
            routineString = routineString.trim();
            if (routineString.length() != 3) {
                DriverStation.reportError("Invalid sequence string: " + sequenceString, false);
                continue;
            }
            char reefLetter = Character.toUpperCase(routineString.charAt(0));
            Level reefLevel = mapLevelStringToReefLevel(routineString.charAt(1));
            char nextCoralStation = Character.toUpperCase(routineString.charAt(2));

            ReefPosition reefPosition = mapReefLetterToSide(reefLetter);
            String pathSegment = mapReefLetterToPathSegment(reefLetter);

            String coralToReefPathPositionName;
            String reefToCoralStationPathName;
            if (previousCoralStation == 'L') {
                coralToReefPathPositionName = "Left " + pathSegment + " Position";
            } else {
                coralToReefPathPositionName = "Right " + pathSegment + " Position";
            }
            if (nextCoralStation == 'L') {
                reefToCoralStationPathName = reefLetter + " to Left";
            } else {
                reefToCoralStationPathName = reefLetter + " to Right";
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

            Command scoreCoralCommand = new ScoreCoralCommand(reefLevel, driveSubsystem, visionSubsystem, elevatorSubsystem, coralSubsystem,
                    reefPosition);

            Command intakeCoralCommand = new IntakeCommand(elevatorSubsystem, coralSubsystem, intakeSubsystem);

            // Add the commands for this instruction to the sequence.
            autonomousSequence.addCommands(
                    intakeCoralCommand,
                    coralStationToReefPositionCommand,
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
     * Converts the starting to the corresponding positioning path segment.
     *
     * @param startingPosition The letter representing the reef section.
     * @return The corresponding path segment string.
     */
    private String getStartToReefPathName(char startingPosition) {
        switch (startingPosition) {
            case 'r':
                return "Start to CD";
            case 'l':
                return "Start to KL";
            case 'c':
                return "Start to GH";
            default:
                return "";
        }
    }

    /**
     * Returns the starting reef-to-coral-station path name based on the starting
     * position and coral station.
     *
     * @param startingPosition The starting position character (r, l, or c).
     * @param coralStation     The starting coral station character (L or R).
     * @return The corresponding path name
     */
    private String getStartingReefToCoralStationPathName(char startingPosition, char coralStation) {
        switch (startingPosition) {
            case 'r':
                return (coralStation == 'R') ? "D to Right" : (coralStation == 'L' ? "C to Left" : "");
            case 'l':
                return (coralStation == 'R') ? "L to Right" : (coralStation == 'L' ? "K to Left" : "");
            case 'c':
                return (coralStation == 'R') ? "H to Right" : (coralStation == 'L' ? "G to Left" : "");
            default:
                return "";
        }
    }

    /**
     * Converts the reef letter to the corresponding positioning path segment.
     *
     * @param reefLetter The letter representing the reef section.
     * @return The corresponding path segment string.
     */
    private static ReefPosition mapReefLetterToSide(char reefLetter) {
        switch (reefLetter) {
            case 'A':
            case 'C':
            case 'E':
            case 'G':
            case 'I':
            case 'K':
                return ReefPosition.LEFT;
            default:
                return ReefPosition.RIGHT;
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

    /**
     * Validates whether the given sequence string is in the correct format.
     *
     * Valid format:
     * startingCommand : routine [, routine]*
     *
     * - startingCommand: [rcl][1-4][lr] (case-insensitive)
     * - routine: [A-L][1-4][lr] (case-insensitive)
     * - Commands may be separated by optional whitespace.
     *
     * @param sequence the input sequence string
     * @return true if the sequence is valid; false otherwise.
     */
    public static boolean isValidAuto(String sequence) {
        if (sequence == null) {
            return false;
        }
        // The regex uses an inline case-insensitive flag (?i)
        // Breakdown:
        // ^\s* : optional whitespace at the beginning
        // [rcl] : starting position (r, c, or l)
        // [1-4] : reef level (1-4)
        // [lr] : coral station (l or r)
        // \s*:\s* : a colon with optional whitespace around it
        // [a-l] : reef letter (A-L, case-insensitive)
        // [1-4] : reef level (1-4)
        // [lr] : coral station (l or r)
        // (?:\s*,\s*[a-l][1-4][lr])* : zero or more additional routine commands
        // separated by commas
        // \s*$ : optional whitespace until the end of the string
        String regex = "(?i)^\\s*[rcl][1-4][lr]\\s*:\\s*[a-l][1-4][lr](?:\\s*,\\s*[a-l][1-4][lr])*\\s*$";
        return sequence.matches(regex);
    }

}
