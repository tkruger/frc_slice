// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.modes.Pathplannerless;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.Constants;
import frc.robot.auto.AutoSelector;
import frc.robot.commands.InstantCalibrationCommand;
import frc.robot.commands.Drivetrain.*;
import frc.robot.commands.Intake.TimedRunMandiblesCommand;
import frc.robot.commands.Limelight.LimelightXAlignmentCommand;
import frc.robot.commands.Wrist.SetWristPosition;
import frc.robot.commands.Wrist.TimedWristRunCommand;
import frc.robot.commands.sequences.PlaceHighRowSequence;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreConeAndCubeMode extends SequentialCommandGroup {
  /** Creates a new ScoreOnePieceMobilityThenAlignMode. */
  public ScoreConeAndCubeMode(AutoSelector.StartingPosition startPosition, Drivetrain drive, Elevator elevator, Wrist wrist, Intake intake, Limelight limelight) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    double turnAngle, turnAngle2 = 0;
    double aprilTag;

    switch(startPosition) {
      case BLUE_COMMUNITY_LEFT:
        //turnAngle = 175;
        //turnAngle2 = 185;
        aprilTag = 6;
        break;
      case RED_COMMUNITY_RIGHT:
        //turnAngle = 185;
        //turnAngle2 = 175;
        aprilTag = 3;
        break;
      default:
        turnAngle = 180;
        aprilTag = 0;
    }

    //CalibrateElevatorCommand calibrateElevator = new CalibrateElevatorCommand(elevator);
    //ResetAngleCommand resetWristAngle = new ResetAngleCommand(wrist);
    InstantCalibrationCommand calibrateElevatorAndWrist = new InstantCalibrationCommand(elevator, wrist);
    PlaceHighRowSequence placePiece = new PlaceHighRowSequence(elevator, wrist, intake);
    PlaceHighRowSequence placeSecondPiece = new PlaceHighRowSequence(elevator, wrist, intake);
    AutonomousTimedDriveStraightCommand mobility = new AutonomousTimedDriveStraightCommand(drive, 0.5, 3.25); //3.25
    AutonomousTimedDriveCommand pickUpDrive = new AutonomousTimedDriveCommand(drive, -0.4, 0, 1.7);
    AutonomousTimedDriveStraightCommand driveHalfBack = new AutonomousTimedDriveStraightCommand(drive, 0.5, 1.5);
    AutonomousTimedDriveStraightCommand returnToGrid = new AutonomousTimedDriveStraightCommand(drive, -0.5, 2);
    QuickTurnSequence quickTurn = new QuickTurnSequence(drive);
    LimelightXAlignmentCommand alignWithCube = new LimelightXAlignmentCommand(limelight, drive);
    LimelightXAlignmentCommand alignWithApriltag = new LimelightXAlignmentCommand(limelight, drive, aprilTag);
    //VariableQuickTurnSequence turnBack = new VariableQuickTurnSequence(drive, turnAngle2);
    TimedRunMandiblesCommand confirmMandiblesOpen = new TimedRunMandiblesCommand(intake, false, 0.3);
    TimedRunMandiblesCommand closeMandibles = new TimedRunMandiblesCommand(intake, true, 0.6);
    SetWristPosition setWristGround = new SetWristPosition(wrist, Constants.States.LOW_ROW_GROUND_STATE.wristAngle);
    TimedWristRunCommand runWristUp = new TimedWristRunCommand(wrist, true, 0.3);
    SetWristPosition stowWrist = new SetWristPosition(wrist, Constants.States.TRAVEL_STATE.wristAngle);

    //ParallelRaceGroup calibrateElevatorAndWrist = new ParallelCommandGroup(calibrateElevator, resetWristAngle).withTimeout(2);
    ParallelCommandGroup alignGroup = new ParallelCommandGroup(confirmMandiblesOpen, setWristGround);
    ParallelCommandGroup stowWhileBacking = new ParallelCommandGroup(driveHalfBack, stowWrist);

    addCommands(
      calibrateElevatorAndWrist,
      placePiece,
      mobility,
      quickTurn,
      alignWithCube,
      alignGroup,
      runWristUp,
      pickUpDrive,
      closeMandibles,
      stowWhileBacking,
      alignWithApriltag,
      returnToGrid,
      placeSecondPiece
    );

  }

}
