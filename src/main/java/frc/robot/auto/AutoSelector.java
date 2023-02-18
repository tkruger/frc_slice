package frc.robot.auto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auto.modes.TwoGamePiecesThenEngageMode;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
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

        TWO_GAME_PIECES_THEN_ENGAGE

    }

    private StartingPosition storedStartingPosition;
    private DesiredMode storedDesiredMode;

    private SendableChooser<StartingPosition> startingPositionChooser;
    private SendableChooser<DesiredMode> modeChooser;

    private Optional<SequentialCommandGroup> autoMode = Optional.empty();

    private final Drivetrain m_drivetrain;
    private final Elevator m_elevator;
    private final Wrist m_wrist;
    private final Intake m_intake;

    public AutoSelector(Drivetrain drivetrain, Elevator elevator, Wrist wrist, Intake intake) {

        m_drivetrain = drivetrain;
        m_elevator = elevator;
        m_wrist = wrist;
        m_intake = intake;

        startingPositionChooser = new SendableChooser<>();

        startingPositionChooser.setDefaultOption("Blue Left", StartingPosition.BLUE_COMMUNITY_LEFT);

        startingPositionChooser.addOption("Blue Middle", StartingPosition.BLUE_COMMUNITY_MIDDLE);
        startingPositionChooser.addOption("Blue Right", StartingPosition.BLUE_COMMUNITY_RIGHT);
        startingPositionChooser.addOption("Red Left", StartingPosition.RED_COMMUNITY_LEFT);
        startingPositionChooser.addOption("Red Middle", StartingPosition.RED_COMMUNITY_MIDDLE);
        startingPositionChooser.addOption("Red Right", StartingPosition.RED_COMMUNITY_RIGHT);

        SmartDashboard.putData("Starting Position", startingPositionChooser);

        modeChooser = new SendableChooser<>();

        modeChooser.setDefaultOption("Two Game Pieces Then Engage", DesiredMode.TWO_GAME_PIECES_THEN_ENGAGE);

        SmartDashboard.putData("Auto Mode", modeChooser);

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

            case TWO_GAME_PIECES_THEN_ENGAGE:
                return Optional.of(new TwoGamePiecesThenEngageMode(position, m_drivetrain, m_elevator, m_wrist, m_intake));
            default:
                break;

        }

        System.err.println("No valid auto mode found for " + mode);
        return Optional.empty();

    }

    public void reset() {

        autoMode = Optional.empty();
        storedDesiredMode = null;

    }

    public void outputToSmartDashboard() {

        SmartDashboard.putString("Selected Auto Mode", storedDesiredMode.name());
        SmartDashboard.putString("Selected Starting Position", storedStartingPosition.name());
        
    }

    public SequentialCommandGroup getAutoMode() {

        return autoMode.get();

    }

}
