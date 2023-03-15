// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.modes;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.auto.AutoSelector;
import frc.robot.auto.paths.GridToChargeStationPath;
import frc.robot.auto.paths.GridToGamePiecePath;
import frc.robot.auto.paths.GamePieceToGridPath;
import frc.robot.auto.sequences.Field2dTrajectoryFollowerSequence;
import frc.robot.commands.Drivetrain.ChargeStationBalancePIDCommand;
import frc.robot.commands.Drivetrain.QuickTurnPIDCommand;
import frc.robot.commands.Elevator.CalibrateElevatorCommand;
import frc.robot.commands.Wrist.ResetAngleCommand;
import frc.robot.commands.sequences.PickUpGamePieceGroundSequence;
import frc.robot.commands.sequences.PlaceHighRowSequence;
import frc.robot.commands.sequences.PlaceCubeMidRowSequence;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreTwoGamePiecesThenEngageMode extends SequentialCommandGroup {
  /** Creates a new OneGamePieceThenEngageMode. */
  public ScoreTwoGamePiecesThenEngageMode(AutoSelector.StartingPosition startPosition, Drivetrain drive, Elevator elevator, Wrist wrist, Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    CalibrateElevatorCommand calibrateElevator = new CalibrateElevatorCommand(elevator);
    ResetAngleCommand resetWristAngle = new ResetAngleCommand(wrist);
    PlaceHighRowSequence placeCube1 = new PlaceHighRowSequence(elevator, wrist, intake);
    GridToGamePiecePath gridToGamePiece = new GridToGamePiecePath(startPosition);
    QuickTurnPIDCommand quickTurn1 = new QuickTurnPIDCommand(drive);
    PickUpGamePieceGroundSequence pickUpGamePiece = new PickUpGamePieceGroundSequence(elevator, wrist, intake);
    QuickTurnPIDCommand quickTurn2 = new QuickTurnPIDCommand(drive);
    GamePieceToGridPath gamePieceToGrid = new GamePieceToGridPath(startPosition);
    PlaceCubeMidRowSequence placeCube2 = new PlaceCubeMidRowSequence(elevator, wrist, intake);
    GridToChargeStationPath gridToChargeStation = new GridToChargeStationPath(startPosition);
    ChargeStationBalancePIDCommand chargeStationBalance = new ChargeStationBalancePIDCommand(drive);

    ParallelCommandGroup calibrateElevatorAndWrist = new ParallelCommandGroup(calibrateElevator, resetWristAngle);
    Field2dTrajectoryFollowerSequence trajectory1 = new Field2dTrajectoryFollowerSequence(drive, gridToGamePiece, gridToGamePiece.trajectory.getInitialPose());
    Field2dTrajectoryFollowerSequence trajectory2 = new Field2dTrajectoryFollowerSequence(drive, gamePieceToGrid);
    Field2dTrajectoryFollowerSequence trajectory3 = new Field2dTrajectoryFollowerSequence(drive, gridToChargeStation);

    addCommands(
      calibrateElevatorAndWrist,
      placeCube1,
      trajectory1,
      quickTurn1,
      pickUpGamePiece,
      quickTurn2,
      trajectory2,
      placeCube2,
      trajectory3,
      chargeStationBalance
    );
    
  }
  
}
