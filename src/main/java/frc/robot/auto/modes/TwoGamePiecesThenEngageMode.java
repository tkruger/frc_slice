// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.modes;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.auto.AutoSelector;
import frc.robot.auto.paths.GridToChargeStationPath;
import frc.robot.auto.paths.GridToGamePiecePath;
import frc.robot.auto.paths.GamePieceToGridPath;
import frc.robot.auto.sequences.Field2dTrajectoryFollowerSequence;
import frc.robot.commands.Drivetrain.ChargeStationBalancePIDCommand;
import frc.robot.commands.Drivetrain.QuickTurnPIDCommand;
import frc.robot.commands.sequences.PickUpGamePieceSequence;
import frc.robot.commands.sequences.PlaceGamePieceMidRowSequence;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoGamePiecesThenEngageMode extends SequentialCommandGroup {
  /** Creates a new OneGamePieceThenEngageMode. */
  public TwoGamePiecesThenEngageMode(AutoSelector.StartingPosition startPosition, Drivetrain drive, Elevator elevator, Wrist wrist, Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    PlaceGamePieceMidRowSequence placeGamePiece1 = new PlaceGamePieceMidRowSequence(elevator, wrist, intake);
    GridToGamePiecePath gridToGamePiece = new GridToGamePiecePath(startPosition, drive);
    QuickTurnPIDCommand quickTurn1 = new QuickTurnPIDCommand(drive);
    PickUpGamePieceSequence pickUpGamePiece = new PickUpGamePieceSequence(elevator, wrist, intake);
    QuickTurnPIDCommand quickTurn2 = new QuickTurnPIDCommand(drive);
    GamePieceToGridPath gamePieceToGrid = new GamePieceToGridPath(startPosition, drive);
    PlaceGamePieceMidRowSequence placeGamePiece2 = new PlaceGamePieceMidRowSequence(elevator, wrist, intake);
    GridToChargeStationPath gridToChargeStation = new GridToChargeStationPath(startPosition, drive);
    ChargeStationBalancePIDCommand chargeStationBalance = new ChargeStationBalancePIDCommand(drive);

    Field2dTrajectoryFollowerSequence trajectory1 = new Field2dTrajectoryFollowerSequence(drive, gridToGamePiece, gridToGamePiece.trajectory.getInitialPose());
    Field2dTrajectoryFollowerSequence trajectory2 = new Field2dTrajectoryFollowerSequence(drive, gamePieceToGrid);
    Field2dTrajectoryFollowerSequence trajectory3 = new Field2dTrajectoryFollowerSequence(drive, gridToChargeStation);

    addCommands(
      placeGamePiece1,
      trajectory1,
      quickTurn1,
      pickUpGamePiece,
      quickTurn2,
      trajectory2,
      placeGamePiece2,
      trajectory3,
      chargeStationBalance
    );
    
  }
  
}
