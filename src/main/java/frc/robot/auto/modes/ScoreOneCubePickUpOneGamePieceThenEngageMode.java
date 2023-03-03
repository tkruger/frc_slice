// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.modes;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.ColorSensor;
import frc.robot.auto.AutoSelector;
import frc.robot.auto.paths.GamePieceToChargeStationPath;
import frc.robot.auto.paths.GridToGamePiecePath;
import frc.robot.auto.sequences.Field2dTrajectoryFollowerSequence;
import frc.robot.commands.Drivetrain.ChargeStationBalancePIDCommand;
import frc.robot.commands.Drivetrain.QuickTurnPIDCommand;
import frc.robot.commands.sequences.PickUpGamePieceGroundSequence;
import frc.robot.commands.sequences.PlaceGamePieceMidRowSequence;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreOneCubePickUpOneGamePieceThenEngageMode extends SequentialCommandGroup {
  /** Creates a new ScoreOneCubePickUpGamePieceThenEngageMode. */
  public ScoreOneCubePickUpOneGamePieceThenEngageMode(AutoSelector.StartingPosition startPosition, Drivetrain drive, Elevator elevator, Wrist wrist, Intake intake, ColorSensor colorSensor) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    PlaceGamePieceMidRowSequence placeCube1 = new PlaceGamePieceMidRowSequence(elevator, wrist, intake);
    GridToGamePiecePath gridToGamePiece = new GridToGamePiecePath(startPosition);
    QuickTurnPIDCommand quickTurn1 = new QuickTurnPIDCommand(drive);
    PickUpGamePieceGroundSequence pickUpGamePiece = new PickUpGamePieceGroundSequence(elevator, wrist, intake, colorSensor);
    QuickTurnPIDCommand quickTurn2 = new QuickTurnPIDCommand(drive);
    GamePieceToChargeStationPath gamePieceToChargeStation = new GamePieceToChargeStationPath(startPosition);
    ChargeStationBalancePIDCommand chargeStationBalance = new ChargeStationBalancePIDCommand(drive);

    Field2dTrajectoryFollowerSequence trajectory1 = new Field2dTrajectoryFollowerSequence(drive, gridToGamePiece, gridToGamePiece.trajectory.getInitialPose());
    Field2dTrajectoryFollowerSequence trajectory2 = new Field2dTrajectoryFollowerSequence(drive, gamePieceToChargeStation);

    addCommands(
      placeCube1,
      trajectory1,
      quickTurn1,
      pickUpGamePiece,
      quickTurn2,
      trajectory2,
      chargeStationBalance
    );

  }

}