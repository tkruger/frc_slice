package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.auto.modes.TestAutoMode;
import frc.robot.subsystems.SwerveDrivetrain;

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

        TEST_AUTO

    }

    private StartingPosition storedStartingPosition;
    private DesiredMode storedDesiredMode;

    private SendableChooser<StartingPosition> startingPositionChooser;
    private SendableChooser<DesiredMode> modeChooser;

    private Optional<SequentialCommandGroup> autoMode = Optional.empty();

    private Optional<Pose2d> initialAutoPose;

    private double initialAutoPoseXOffset;
    private double initialAutoPoseYOffset;
    private double initialAutoPoseRotationOffset;

    private final SwerveDrivetrain m_swerveDrivetrain;

    private final ShuffleboardTab autoTab;

    private final GenericEntry selectedAutoModeWidget;
    private final GenericEntry selectedStartingPositionWidget;

    private final GenericEntry autoPoseXOffsetWidget;
    private final GenericEntry autoPoseYOffsetWidget;
    private final GenericEntry autoPoseRotationOffsetWidget;

    public AutoSelector(SwerveDrivetrain drivetrain) {

        m_swerveDrivetrain = drivetrain;

        startingPositionChooser = new SendableChooser<StartingPosition>();

        startingPositionChooser.setDefaultOption("Blue Left", StartingPosition.BLUE_COMMUNITY_LEFT);

        startingPositionChooser.addOption("Blue Middle", StartingPosition.BLUE_COMMUNITY_MIDDLE);
        startingPositionChooser.addOption("Blue Right", StartingPosition.BLUE_COMMUNITY_RIGHT);
        startingPositionChooser.addOption("Red Left", StartingPosition.RED_COMMUNITY_LEFT);
        startingPositionChooser.addOption("Red Middle", StartingPosition.RED_COMMUNITY_MIDDLE);
        startingPositionChooser.addOption("Red Right", StartingPosition.RED_COMMUNITY_RIGHT);

        modeChooser = new SendableChooser<DesiredMode>();

        modeChooser.setDefaultOption("Test Auto", DesiredMode.TEST_AUTO);

        autoTab = Shuffleboard.getTab("Auto Tab");

        autoTab.add("Auto Mode", modeChooser).withPosition(2, 0).withSize(2, 1);
        autoTab.add("Starting Position", startingPositionChooser).withPosition(5, 0).withSize(2, 1);

        selectedAutoModeWidget = 
        autoTab.add("Selected Auto Mode", "Score Two Game Pieces Then Engage").
        withPosition(2, 1).
        withSize(2, 1).
        getEntry();
        selectedStartingPositionWidget = 
        autoTab.add("Selected Starting Position", "Blue Left").
        withPosition(5, 1).
        withSize(2, 1).
        getEntry();

        autoPoseXOffsetWidget = 
        autoTab.add("Initial Auto Pose X Offset", initialAutoPoseXOffset).
        withPosition(1, 2).
        withSize(2, 1).
        getEntry();

        autoPoseYOffsetWidget = 
        autoTab.add("Initial Auto Pose Y Offset", initialAutoPoseYOffset).
        withPosition(6, 2).
        withSize(2, 1).
        getEntry();

        autoPoseRotationOffsetWidget = 
        autoTab.add("Initial Auto Pose Rotation Offset", initialAutoPoseRotationOffset).
        withPosition(3, 2).
        withSize(3, 1).
        getEntry();

    }

    public void updateModeCreator() {

        StartingPosition startingPosition = startingPositionChooser.getSelected();
        DesiredMode desiredMode = modeChooser.getSelected();

        if(storedStartingPosition != startingPosition || storedDesiredMode != desiredMode) {
            
            System.out.println("Auto selection changed, updating creator; Starting Position: " + startingPosition.name()
            + ", Desired Mode: " + desiredMode.name());

            autoMode = getAutoModeForParams(startingPosition, desiredMode);

        }

        storedStartingPosition = startingPosition;
        storedDesiredMode = desiredMode;

    }

    private Optional<SequentialCommandGroup> getAutoModeForParams(StartingPosition position, DesiredMode mode) {

        switch(mode) {

            case TEST_AUTO:
                return Optional.of(new TestAutoMode(position, m_swerveDrivetrain));
            default:
                break;
    
        }

        System.err.println("No valid auto mode found for " + mode);
        return Optional.empty();

    }

    /*public void updateInitialAutoPoseOffset() {

        Pose2d botPose = Limelight.getLastBotPoseBlue();

        StartingPosition startingPosition = startingPositionChooser.getSelected();
        DesiredMode desiredMode = modeChooser.getSelected();

        if(storedStartingPosition != startingPosition || storedDesiredMode != desiredMode) {

            switch(storedDesiredMode) {

                case SCORE_TWO_GAME_PIECES_THEN_ENGAGE:
                    initialAutoPose = Optional.of(new GridToGamePiecePath(storedStartingPosition).trajectory.getInitialPose());
                    break;
                case SCORE_ONE_CUBE_GO_OUT_THEN_ENGAGE:
                    initialAutoPose = Optional.of(new GridOutOfCommunityToChargeStationPath(storedStartingPosition).trajectory.getInitialPose());
                    break;
                case SCORE_ONE_CONE_HIGH_ROW:
                    System.out.println("No initial pose is available for the 'ScoreConeHighRow' mode");
                    initialAutoPose = Optional.of(botPose);
                    break;
                case SCORE_ONE_CUBE_PICK_UP_ONE_GAME_PIECE_THEN_ENGAGE:
                    initialAutoPose = Optional.of(new GridToGamePiecePath(storedStartingPosition).trajectory.getInitialPose());
                    break;
                case SCORE_ONE_PIECE_THEN_MOBILITY:
                    System.out.println("No initial pose is available for the 'ScoreOnePieceThenMobility' mode");
                    initialAutoPose = Optional.of(botPose);
                    break;
                case SCORE_ONE_GAME_PIECE_THEN_ENGAGE:
                    System.out.println("No initial pose is available for the 'ScoreConeHighRow' mode");
                    initialAutoPose = Optional.of(botPose);
                    break;
                case SCORE_ONE_GAME_PIECE_MOBILITY_THEN_ALIGN:
                    System.out.println("No initial pose is available for the 'Score' mode");
                    initialAutoPose = Optional.of(botPose);
                    break;
                default:
                    System.err.println("No valid initial auto pose found for " + storedDesiredMode);
                    initialAutoPose = Optional.empty();
                    break;
                
            }

        }

        if(botPose != null && initialAutoPose != null) {

            initialAutoPoseXOffset = initialAutoPose.get().getX() - botPose.getX();
            initialAutoPoseYOffset = initialAutoPose.get().getY() - botPose.getY();
            initialAutoPoseRotationOffset = initialAutoPose.get().getRotation().getDegrees() - botPose.getRotation().getDegrees();

        }

    }*/

    public void reset() {

        autoMode = Optional.empty();
        storedDesiredMode = null;

        initialAutoPose = Optional.empty();

    }

    public void outputToShuffleboard() {

        selectedAutoModeWidget.setString(storedDesiredMode.name());
        selectedStartingPositionWidget.setString(storedStartingPosition.name());

        if(initialAutoPose != null) {

            autoPoseXOffsetWidget.setDouble(initialAutoPoseXOffset);
            autoPoseYOffsetWidget.setDouble(initialAutoPoseYOffset);
            autoPoseRotationOffsetWidget.setDouble(initialAutoPoseRotationOffset);

        }

    }

    public SequentialCommandGroup getAutoMode() {

        return autoMode.get();

    }

}
