package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.auto.modes.OneCubeGoOutThenEngageMode;
import frc.robot.auto.modes.TwoCubesThenEngageMode;
import frc.robot.auto.paths.GridOutOfCommunityToChargeStationPath;
import frc.robot.auto.paths.GridToGamePiecePath;
import frc.robot.commands.sequences.PlaceGamePieceHighRowSequence;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Wrist;

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

        TWO_CUBES_THEN_ENGAGE,
        ONE_CUBE_GO_OUT_THEN_ENGAGE,
        SCORE_CONE_HIGH_ROW,

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

    private final Drivetrain m_drivetrain;
    private final Elevator m_elevator;
    private final Wrist m_wrist;
    private final Intake m_intake;
    private final ColorSensor m_colorSensor;

    private final ShuffleboardTab autoTab;

    private final SimpleWidget selectedAutoModeWidget;
    private final SimpleWidget selectedStartingPositionWidget;

    private final SimpleWidget autoPoseXOffsetWidget;
    private final SimpleWidget autoPoseYOffsetWidget;
    private final SimpleWidget autoPoseRotationOffsetWidget;

    public AutoSelector(Drivetrain drivetrain, Elevator elevator, Wrist wrist, Intake intake, ColorSensor colorSensor) {

        m_drivetrain = drivetrain;
        m_elevator = elevator;
        m_wrist = wrist;
        m_intake = intake;
        m_colorSensor = colorSensor;

        startingPositionChooser = new SendableChooser<>();

        startingPositionChooser.setDefaultOption("Blue Left", StartingPosition.BLUE_COMMUNITY_LEFT);

        startingPositionChooser.addOption("Blue Middle", StartingPosition.BLUE_COMMUNITY_MIDDLE);
        startingPositionChooser.addOption("Blue Right", StartingPosition.BLUE_COMMUNITY_RIGHT);
        startingPositionChooser.addOption("Red Left", StartingPosition.RED_COMMUNITY_LEFT);
        startingPositionChooser.addOption("Red Middle", StartingPosition.RED_COMMUNITY_MIDDLE);
        startingPositionChooser.addOption("Red Right", StartingPosition.RED_COMMUNITY_RIGHT);

        modeChooser = new SendableChooser<>();

        modeChooser.setDefaultOption("Two Cubes Then Engage", DesiredMode.TWO_CUBES_THEN_ENGAGE);

        modeChooser.addOption("One Cube Go Out Then Engage", DesiredMode.ONE_CUBE_GO_OUT_THEN_ENGAGE);
        modeChooser.addOption("Score Cone High Row", DesiredMode.SCORE_CONE_HIGH_ROW);

        autoTab = Shuffleboard.getTab("Auto Tab");

        autoTab.add("Auto Mode", modeChooser).withPosition(2, 0).withSize(2, 1);
        autoTab.add("Starting Position", startingPositionChooser).withPosition(4, 0).withSize(2, 1);

        selectedAutoModeWidget = 
        autoTab.add("Selected Auto Mode", "Two Cubes Then Engage").
        withPosition(2, 1).
        withSize(2, 1);
        selectedStartingPositionWidget = 
        autoTab.add("Selected Starting Position", "Blue Left").
        withPosition(4, 1).
        withSize(2, 1);

        autoPoseXOffsetWidget = 
        autoTab.add("Initial Auto Pose X Offset", initialAutoPoseXOffset).
        withPosition(1, 2).
        withSize(2, 1);

        autoPoseYOffsetWidget = 
        autoTab.add("Initial Auto Pose Y Offset", initialAutoPoseYOffset).
        withPosition(5, 2).
        withSize(2, 1);

        autoPoseRotationOffsetWidget = 
        autoTab.add("Initial Auto Pose Rotation Offset", initialAutoPoseRotationOffset).
        withPosition(3, 2).
        withSize(2, 1);

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

            case TWO_CUBES_THEN_ENGAGE:
                return Optional.of(new TwoCubesThenEngageMode(position, m_drivetrain, m_elevator, m_wrist, m_intake, m_colorSensor));
            case ONE_CUBE_GO_OUT_THEN_ENGAGE:
                return Optional.of(new OneCubeGoOutThenEngageMode(position, m_drivetrain, m_elevator, m_wrist, m_intake));
            case SCORE_CONE_HIGH_ROW:
                return Optional.of(new PlaceGamePieceHighRowSequence(m_elevator, m_wrist, m_intake));
            default:
                break;
    
        }

        System.err.println("No valid auto mode found for " + mode);
        return Optional.empty();

    }

    public void updateInitialAutoPoseOffset() {

        Pose2d botPoseBlue = Limelight.getBotPoseBlue();

        switch(storedDesiredMode) {

            case TWO_CUBES_THEN_ENGAGE:
                initialAutoPose = Optional.of(new GridToGamePiecePath(storedStartingPosition, m_drivetrain).trajectory.getInitialPose());
                break;
            case ONE_CUBE_GO_OUT_THEN_ENGAGE:
                initialAutoPose = Optional.of(new GridOutOfCommunityToChargeStationPath(storedStartingPosition, m_drivetrain).trajectory.getInitialPose());
                break;
            case SCORE_CONE_HIGH_ROW:
                System.out.println("No initial pose is available for the 'ScoreConeHighRow' mode");
                initialAutoPose = Optional.of(Limelight.getBotPoseBlue());
                break;
            default:
                System.err.println("No valid initial auto pose found for " + storedDesiredMode);
                initialAutoPose = Optional.empty();
                break;

        }

        if(botPoseBlue != null) {

            initialAutoPoseXOffset = initialAutoPose.get().getX() - botPoseBlue.getX();
            initialAutoPoseYOffset = initialAutoPose.get().getY() - botPoseBlue.getY();
            initialAutoPoseRotationOffset = initialAutoPose.get().getRotation().getDegrees() - botPoseBlue.getRotation().getDegrees();

        }

    }

    public void reset() {

        autoMode = Optional.empty();
        storedDesiredMode = null;

        initialAutoPose = Optional.empty();

    }

    public void outputToShuffleboard() {

        selectedAutoModeWidget.getEntry().setString(storedDesiredMode.name());
        selectedStartingPositionWidget.getEntry().setString(storedStartingPosition.name());

        autoPoseXOffsetWidget.getEntry().setDouble(initialAutoPoseXOffset);
        autoPoseYOffsetWidget.getEntry().setDouble(initialAutoPoseYOffset);
        autoPoseRotationOffsetWidget.getEntry().setDouble(initialAutoPoseRotationOffset);

    }

    public SequentialCommandGroup getAutoMode() {

        return autoMode.get();

    }

}
