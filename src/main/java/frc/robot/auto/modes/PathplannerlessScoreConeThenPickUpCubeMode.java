// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.modes;

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
public class PathplannerlessScoreConeThenPickUpCubeMode extends SequentialCommandGroup {
  /** Creates a new ScoreOnePieceMobilityThenAlignMode. */
  public PathplannerlessScoreConeThenPickUpCubeMode(AutoSelector.StartingPosition startPosition, Drivetrain drive, Elevator elevator, Wrist wrist, Intake intake, Limelight limelight) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    double turnAngle, turnAngle2 = 0;

    switch(startPosition) {
      case BLUE_COMMUNITY_LEFT:
        turnAngle = 175;
        turnAngle2 = 185;
        break;
      case RED_COMMUNITY_RIGHT:
        turnAngle = 185;
        turnAngle2 = 175;
        break;
      default:
        turnAngle = 180;
    }

    InstantCalibrationCommand calibrateElevatorAndWrist = new InstantCalibrationCommand(elevator, wrist);
    PlaceHighRowSequence placePiece = new PlaceHighRowSequence(elevator, wrist, intake);
    AutonomousTimedDriveStraightCommand mobility = new AutonomousTimedDriveStraightCommand(drive, 0.5, 3.35); //3.25
    AutonomousTimedDriveCommand pickUpDrive = new AutonomousTimedDriveCommand(drive, -0.4, 0, 1.3);
    VariableQuickTurnSequence quickTurn = new VariableQuickTurnSequence(drive, turnAngle);
    LimelightXAlignmentCommand alignWithCube = new LimelightXAlignmentCommand(limelight, drive);
    VariableQuickTurnSequence turnBack = new VariableQuickTurnSequence(drive, turnAngle2);
    TimedRunMandiblesCommand confirmMandiblesOpen = new TimedRunMandiblesCommand(intake, false, 0.3);
    TimedRunMandiblesCommand closeMandibles = new TimedRunMandiblesCommand(intake, true, 0.6);
    SetWristPosition setWristGround = new SetWristPosition(wrist, Constants.States.LOW_ROW_GROUND_STATE.wristAngle);
    TimedWristRunCommand runWristUp = new TimedWristRunCommand(wrist, true, 0.3);
    SetWristPosition stowWrist = new SetWristPosition(wrist, Constants.States.TRAVEL_STATE.wristAngle);

    ParallelCommandGroup openWhileTurning = new ParallelCommandGroup(quickTurn, confirmMandiblesOpen);
    ParallelCommandGroup stowWhileTurning = new ParallelCommandGroup(turnBack, stowWrist);

    addCommands(
      calibrateElevatorAndWrist,
      placePiece,
      mobility,
      openWhileTurning,
      alignWithCube,
      setWristGround,
      runWristUp,
      pickUpDrive,
      closeMandibles,
      stowWhileTurning
    );

  }

}
