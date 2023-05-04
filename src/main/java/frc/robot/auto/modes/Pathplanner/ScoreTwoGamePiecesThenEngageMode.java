// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.modes.Pathplanner;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.auto.AutoSelector;
import frc.robot.auto.paths.GridToChargeStationPath;
import frc.robot.auto.paths.GridToGamePiecePath;
import frc.robot.auto.paths.GamePieceToGridPath;
//import frc.robot.commands.Drivetrain.sequences.Field2dTrajectoryFollowerSequence;
import frc.robot.commands.Drivetrain.sequences.PPField2dTrajectoryFollowerSequence;
import frc.robot.commands.Drivetrain.ChargeStation.BoardChargeStationCommand;
import frc.robot.commands.Drivetrain.ChargeStation.ChargeStationBalancePIDCommand;
import frc.robot.commands.InstantCalibrationCommand;
import frc.robot.commands.Drivetrain.AutonomousTimedDriveCommand;
import frc.robot.commands.Drivetrain.QuickTurnSequence;
import frc.robot.commands.sequences.PickUpGamePieceGroundSequence;
import frc.robot.commands.sequences.PlaceHighRowSequence;
import frc.robot.commands.sequences.PlaceCubeMidRowSequence;
import frc.robot.subsystems.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreTwoGamePiecesThenEngageMode extends SequentialCommandGroup {
  /** Creates a new ScoreTwoGamePiecesThenEngageMode. */
  public ScoreTwoGamePiecesThenEngageMode(AutoSelector.StartingPosition startPosition, Drivetrain drive, Elevator elevator, Wrist wrist, Intake intake, LEDs leds) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    // CalibrateElevatorCommand calibrateElevator = new CalibrateElevatorCommand(elevator);
    // ResetAngleCommand resetWristAngle = new ResetAngleCommand(wrist);
    InstantCalibrationCommand calibrateElevatorAndWrist = new InstantCalibrationCommand(elevator, wrist);
    PlaceHighRowSequence placeCube1 = new PlaceHighRowSequence(elevator, wrist, intake);
    GridToGamePiecePath gridToGamePiece = new GridToGamePiecePath(startPosition);
    QuickTurnSequence quickTurn1 = new QuickTurnSequence(drive);
    PickUpGamePieceGroundSequence pickUpGamePiece = new PickUpGamePieceGroundSequence(elevator, wrist, intake);
    QuickTurnSequence quickTurn2 = new QuickTurnSequence(drive);
    GamePieceToGridPath gamePieceToGrid = new GamePieceToGridPath(startPosition);
    PlaceCubeMidRowSequence placeCube2 = new PlaceCubeMidRowSequence(elevator, wrist, intake);
    AutonomousTimedDriveCommand driveBack = new AutonomousTimedDriveCommand(drive, 0.5, 0, 0.3);
    QuickTurnSequence quickTurn3 = new QuickTurnSequence(drive);
    GridToChargeStationPath gridToChargeStation = new GridToChargeStationPath(startPosition);
    BoardChargeStationCommand getOnChargeStation = new BoardChargeStationCommand(drive);
    ChargeStationBalancePIDCommand chargeStationBalance = new ChargeStationBalancePIDCommand(drive, leds);

    // ParallelCommandGroup calibrateElevatorAndWrist = new ParallelCommandGroup(calibrateElevator, resetWristAngle);
    PPField2dTrajectoryFollowerSequence trajectory1 = new PPField2dTrajectoryFollowerSequence(drive, gridToGamePiece, gridToGamePiece.trajectory.getInitialPose());
    PPField2dTrajectoryFollowerSequence trajectory2 = new PPField2dTrajectoryFollowerSequence(drive, gamePieceToGrid);
    PPField2dTrajectoryFollowerSequence trajectory3 = new PPField2dTrajectoryFollowerSequence(drive, gridToChargeStation);

    addCommands(
      calibrateElevatorAndWrist,
      placeCube1,
      trajectory1,
      quickTurn1,
      pickUpGamePiece,
      quickTurn2,
      trajectory2,
      placeCube2,
      driveBack,
      quickTurn3,
      trajectory3,
      getOnChargeStation,
      chargeStationBalance
    );
    
  }
  
}
