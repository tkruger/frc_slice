// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.modes;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.Constants;
import frc.robot.auto.AutoSelector;
import frc.robot.commands.Drivetrain.*;
import frc.robot.commands.Elevator.CalibrateElevatorCommand;
import frc.robot.commands.Intake.TimedRunMandiblesCommand;
import frc.robot.commands.Wrist.ResetAngleCommand;
import frc.robot.commands.Wrist.SetWristPosition;
import frc.robot.commands.sequences.PlaceHighRowSequence;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PathplannerlessConeThenCubeMode extends SequentialCommandGroup {
  /** Creates a new ScoreOnePieceMobilityThenAlignMode. */
  public PathplannerlessConeThenCubeMode(AutoSelector.StartingPosition startPosition, Drivetrain drive, Elevator elevator, Wrist wrist, Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    double turnAngle = 0;

    switch(startPosition) {
      case BLUE_COMMUNITY_LEFT:
        turnAngle = 184;
        break;
      case RED_COMMUNITY_RIGHT:
        turnAngle = 176;
        break;
      default:
        turnAngle = 180;
    }

    CalibrateElevatorCommand calibrateElevator = new CalibrateElevatorCommand(elevator);
    ResetAngleCommand resetWristAngle = new ResetAngleCommand(wrist);
    PlaceHighRowSequence placePiece = new PlaceHighRowSequence(elevator, wrist, intake);
    PlaceHighRowSequence placeSecondPiece = new PlaceHighRowSequence(elevator, wrist, intake);
    AutonomousTimedDriveCommand mobility = new AutonomousTimedDriveCommand(drive, 0.5, 0, 3.25);
    AutonomousTimedDriveCommand pickUpDrive = new AutonomousTimedDriveCommand(drive, -0.25, 0, .5);
    AutonomousTimedDriveCommand driveBack = new AutonomousTimedDriveCommand(drive, -0.5, 0, 3.4);
    VariableQuickTurnPIDCommand quickTurn = new VariableQuickTurnPIDCommand(drive, turnAngle);
    VariableQuickTurnPIDCommand turnBack = new VariableQuickTurnPIDCommand(drive, 360 - turnAngle);
    TimedRunMandiblesCommand confirmMandiblesOpen = new TimedRunMandiblesCommand(intake, false, 0.3);
    TimedRunMandiblesCommand closeMandibles = new TimedRunMandiblesCommand(intake, true, 0.5);
    SetWristPosition setWristGround = new SetWristPosition(wrist, Constants.States.LOW_ROW_GROUND_STATE.wristAngle);
    SetWristPosition stowWrist = new SetWristPosition(wrist, Constants.States.TRAVEL_STATE.wristAngle);

    ParallelRaceGroup calibrateElevatorAndWrist = new ParallelCommandGroup(calibrateElevator, resetWristAngle).withTimeout(2);
    ParallelCommandGroup openWhileTurning = new ParallelCommandGroup(quickTurn, confirmMandiblesOpen);
    ParallelCommandGroup stowWhileTurning = new ParallelCommandGroup(turnBack, stowWrist);

    addCommands(
      calibrateElevatorAndWrist,
      placePiece,
      mobility,
      openWhileTurning,
      setWristGround,
      pickUpDrive,
      closeMandibles,
      stowWhileTurning,
      driveBack,
      placeSecondPiece
    );

  }

}
