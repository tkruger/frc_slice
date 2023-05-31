package frc.robot.auto.modes.Demo;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.Drivetrain.*;
import frc.robot.commands.Limelight.*;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class DriveSquareCubeAlignMode extends SequentialCommandGroup {
    
    public DriveSquareCubeAlignMode(Drivetrain drivetrain, Limelight limelight) {
        VariableQuickTurnSequence turn1 = new VariableQuickTurnSequence(drivetrain, 100);
        AutonomousDistanceDriveCommand drive1 = new AutonomousDistanceDriveCommand(drivetrain, -0.4, 1.3);

        VariableQuickTurnSequence turn2 = new VariableQuickTurnSequence(drivetrain, 100);
        AutonomousDistanceDriveCommand drive2 = new AutonomousDistanceDriveCommand(drivetrain, -0.4, 1.3);

        VariableQuickTurnSequence turn3 = new VariableQuickTurnSequence(drivetrain, 100);
        AutonomousDistanceDriveCommand drive3 = new AutonomousDistanceDriveCommand(drivetrain, -0.4, 1.3);

        LimelightXAlignmentCommand alignWithCube = new LimelightXAlignmentCommand(limelight, drivetrain);
        LimelightYAlignmentCommand driveToCube = new LimelightYAlignmentCommand(limelight, drivetrain);
        DriveSquareCubeAlignMode restart = new DriveSquareCubeAlignMode(drivetrain, limelight);

        addCommands(
            alignWithCube,
            driveToCube,
            turn1,
            drive1,
            turn2,
            drive2,
            turn3,
            drive3,
            restart
        );
    }
}
