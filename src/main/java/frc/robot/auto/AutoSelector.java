package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auto.modes.Pathplanner.ScoreOneCubeMobilityThenEngageMode;
import frc.robot.auto.modes.Pathplanner.ScoreOneCubePickUpOneGamePieceThenEngageMode;
import frc.robot.auto.modes.Pathplanner.ScoreTwoGamePiecesMode;
import frc.robot.auto.modes.Pathplanner.ScoreTwoGamePiecesThenEngageMode;
import frc.robot.auto.modes.Pathplannerless.ScoreOneConeHighRowMode;
import frc.robot.auto.paths.GridOutOfCommunityToChargeStationPath;
import frc.robot.auto.paths.GridToGamePiecePath;
import frc.robot.subsystems.Drivetrain;

import java.util.Optional;

public class AutoSelector {
    
    public enum StartingPosition {

        BLUE_COMMUNITY_LEFT, 
        BLUE_COMMUNITY_MIDDLE, 
        BLUE_COMMUNITY_RIGHT, 
        RED_COMMUNITY_LEFT, 
        RED_COMMUNITY_MIDDLE, 
        RED_COMMUNITY_RIGHT

    }

    public enum DesiredMode {
        
        SCORE_ONE_CONE_HIGH_ROW_PATHPLANNERLESS,
        SCORE_ONE_CUBE_MOBILITY_THEN_ENGAGE_PATHPLANNER,
        SCORE_ONE_CUBE_PICK_UP_ONE_GAME_PIECE_THEN_ENGAGE_PATHPLANNER,
        SCORE_TWO_GAME_PIECES_PATHPLANNER,
        SCORE_TWO_GAME_PIECES_THEN_ENGAGE_PATHPLANNER,

    }

    public StartingPosition storedStartingPosition;
    public DesiredMode storedDesiredMode;

    public SendableChooser<StartingPosition> startingPositionChooser;
    public SendableChooser<DesiredMode> modeChooser;

    private Optional<SequentialCommandGroup> autoMode = Optional.empty();

    private Pose2d initialAutoPose;

    public double initialAutoPoseXOffset = 0;
    public double initialAutoPoseYOffset = 0;
    public double initialAutoPoseRotationOffset = 0;

    private final Drivetrain m_drivetrain;

    public AutoSelector(Drivetrain drivetrain) {

        m_drivetrain = drivetrain;

        startingPositionChooser = new SendableChooser<StartingPosition>();

        startingPositionChooser.setDefaultOption("Blue Left", StartingPosition.BLUE_COMMUNITY_LEFT);

        startingPositionChooser.addOption("Blue Middle", StartingPosition.BLUE_COMMUNITY_MIDDLE);
        startingPositionChooser.addOption("Blue Right", StartingPosition.BLUE_COMMUNITY_RIGHT);
        startingPositionChooser.addOption("Red Left", StartingPosition.RED_COMMUNITY_LEFT);
        startingPositionChooser.addOption("Red Middle", StartingPosition.RED_COMMUNITY_MIDDLE);
        startingPositionChooser.addOption("Red Right", StartingPosition.RED_COMMUNITY_RIGHT);

        modeChooser = new SendableChooser<DesiredMode>();

        modeChooser.setDefaultOption("Any - Score One Cone High Row ", DesiredMode.SCORE_ONE_CONE_HIGH_ROW_PATHPLANNERLESS);

        modeChooser.addOption("(Pathplanner) Score One Cube Mobility Then Engage", DesiredMode.SCORE_ONE_CUBE_MOBILITY_THEN_ENGAGE_PATHPLANNER);
        modeChooser.addOption("(Pathplanner) Score One Cube Pick Up One Game Piece Then Engage", DesiredMode.SCORE_ONE_CUBE_PICK_UP_ONE_GAME_PIECE_THEN_ENGAGE_PATHPLANNER);
        modeChooser.addOption("(Pathplanner) Score Two Game Pieces Then Engage", DesiredMode.SCORE_TWO_GAME_PIECES_THEN_ENGAGE_PATHPLANNER);
        modeChooser.addOption("(Pathplanner) Score Two Game Pieces", DesiredMode.SCORE_TWO_GAME_PIECES_PATHPLANNER);

    }

    public void updateAutoSelector() {

        StartingPosition startingPosition = startingPositionChooser.getSelected();
        DesiredMode desiredMode = modeChooser.getSelected();

        if(storedStartingPosition != startingPosition || storedDesiredMode != desiredMode) {
            
            System.out.println("Auto selection changed, updating creator; Starting Position: " + startingPosition.name()
            + ", Desired Mode: " + desiredMode.name());

            autoMode = getAutoModeForParams(startingPosition, desiredMode);

            updateInitialAutoPoseOffset(startingPosition, desiredMode);

        }

        storedStartingPosition = startingPosition;
        storedDesiredMode = desiredMode;

    }

    private Optional<SequentialCommandGroup> getAutoModeForParams(StartingPosition position, DesiredMode mode) {

        switch(mode) {

            case SCORE_ONE_CONE_HIGH_ROW_PATHPLANNERLESS:
                return Optional.of(new ScoreOneConeHighRowMode(/*m_elevator, m_wrist, m_intake*/));
            case SCORE_ONE_CUBE_MOBILITY_THEN_ENGAGE_PATHPLANNER:
                return Optional.of(new ScoreOneCubeMobilityThenEngageMode(position, m_drivetrain));
            case SCORE_ONE_CUBE_PICK_UP_ONE_GAME_PIECE_THEN_ENGAGE_PATHPLANNER:
                return Optional.of(new ScoreOneCubePickUpOneGamePieceThenEngageMode(position, m_drivetrain));
            case SCORE_TWO_GAME_PIECES_PATHPLANNER:
                return Optional.of(new ScoreTwoGamePiecesMode(position, m_drivetrain));
            case SCORE_TWO_GAME_PIECES_THEN_ENGAGE_PATHPLANNER:
                return Optional.of(new ScoreTwoGamePiecesThenEngageMode(position, m_drivetrain));
            default:
                break;
    
        }

        System.err.println("No valid auto mode found for " + mode);
        return Optional.empty();

    }

    public void updateInitialAutoPoseOffset(StartingPosition startingPosition, DesiredMode desiredMode) {

        /*This variable should be should be assigned to Limelight.getLastBotPoseBlue() instead when
        the Limelight subsystem file is added*/
        Pose2d botPose = m_drivetrain.getPose();

        switch(desiredMode) {

            case SCORE_ONE_CONE_HIGH_ROW_PATHPLANNERLESS:
                System.out.println("No initial pose is available for the 'ScoreConeHighRow' mode");
                initialAutoPose = botPose;
                break;
            case SCORE_ONE_CUBE_MOBILITY_THEN_ENGAGE_PATHPLANNER:
                initialAutoPose = new GridOutOfCommunityToChargeStationPath(startingPosition).getTrajectory().getInitialPose();
                break;
            case SCORE_ONE_CUBE_PICK_UP_ONE_GAME_PIECE_THEN_ENGAGE_PATHPLANNER:
                initialAutoPose = new GridToGamePiecePath(startingPosition).getTrajectory().getInitialPose();
                break;
            case SCORE_TWO_GAME_PIECES_PATHPLANNER:
                initialAutoPose = new GridToGamePiecePath(startingPosition).getTrajectory().getInitialPose();
                break;
            case SCORE_TWO_GAME_PIECES_THEN_ENGAGE_PATHPLANNER:
                initialAutoPose = new GridToGamePiecePath(startingPosition).getTrajectory().getInitialPose();
                break;
            default:
                System.err.println("No valid initial auto pose found for " + desiredMode);
                break;
                
            }

        if(botPose != null && initialAutoPose != null) {

            initialAutoPoseXOffset = initialAutoPose.getX() - botPose.getX();
            initialAutoPoseYOffset = initialAutoPose.getY() - botPose.getY();
            initialAutoPoseRotationOffset = initialAutoPose.getRotation().getDegrees() - botPose.getRotation().getDegrees();

        }

    }

    public void reset() {

        autoMode = Optional.empty();
        storedDesiredMode = null;

        initialAutoPose = null;

    }

    public SequentialCommandGroup getAutoMode() {

        return autoMode.get();

    }

    public String getStoredDesiredMode() {

        if(storedDesiredMode != null) {

            return storedDesiredMode.name();

        }
        else {

            return "None Stored";

        }

    }

    public String getStoredStartingPosition() {

        if(storedStartingPosition != null) {

            return storedStartingPosition.name();

        }
        else {

            return "None Stored";

        }

    }

}
