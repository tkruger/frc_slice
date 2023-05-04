// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.modes.Pathplanner;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.auto.AutoSelector;
import frc.robot.auto.paths.GridToChargeStationPath;
import frc.robot.auto.paths.GridToGamePieceWithArcPath;
import frc.robot.auto.paths.GamePieceToGridWithArcPath;
//import frc.robot.commands.Drivetrain.sequences.Field2dTrajectoryFollowerSequence;
import frc.robot.commands.Drivetrain.sequences.PPField2dTrajectoryFollowerSequence;
import frc.robot.commands.InstantCalibrationCommand;
import frc.robot.commands.Drivetrain.ChargeStation.BoardChargeStationCommand;
import frc.robot.commands.Drivetrain.ChargeStation.ChargeStationBalancePIDCommand;
import frc.robot.commands.sequences.PickUpGamePieceGroundSequence;
import frc.robot.commands.sequences.PlaceHighRowSequence;
import frc.robot.commands.sequences.PlaceCubeMidRowSequence;
import frc.robot.subsystems.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreTwoGamePiecesThenEngageWithArcMode extends SequentialCommandGroup {
  /** Creates a new ScoreTwoGamePiecesThenEngageWithArcMode. */
  public ScoreTwoGamePiecesThenEngageWithArcMode(AutoSelector.StartingPosition startPosition, Drivetrain drive, Elevator elevator, Wrist wrist, Intake intake, LEDs leds) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    // CalibrateElevatorCommand calibrateElevator = new CalibrateElevatorCommand(elevator);
    // ResetAngleCommand resetWristAngle = new ResetAngleCommand(wrist);
    InstantCalibrationCommand calibrateElevatorAndWrist = new InstantCalibrationCommand(elevator, wrist);
    PlaceHighRowSequence placeCube1 = new PlaceHighRowSequence(elevator, wrist, intake);
    GridToGamePieceWithArcPath gridToGamePieceWithArc = new GridToGamePieceWithArcPath(startPosition);
    PickUpGamePieceGroundSequence pickUpGamePiece = new PickUpGamePieceGroundSequence(elevator, wrist, intake);
    GamePieceToGridWithArcPath gamePieceToGridWithArc = new GamePieceToGridWithArcPath(startPosition);
    PlaceCubeMidRowSequence placeCube2 = new PlaceCubeMidRowSequence(elevator, wrist, intake);
    GridToChargeStationPath gridToChargeStation = new GridToChargeStationPath(startPosition);
    BoardChargeStationCommand getOnChargeStation = new BoardChargeStationCommand(drive);
    ChargeStationBalancePIDCommand chargeStationBalance = new ChargeStationBalancePIDCommand(drive, leds);

    // ParallelCommandGroup calibrateElevatorAndWrist = new ParallelCommandGroup(calibrateElevator, resetWristAngle);
    PPField2dTrajectoryFollowerSequence trajectory1 = new PPField2dTrajectoryFollowerSequence(drive, gridToGamePieceWithArc, gridToGamePieceWithArc.trajectory.getInitialPose());
    PPField2dTrajectoryFollowerSequence trajectory2 = new PPField2dTrajectoryFollowerSequence(drive, gamePieceToGridWithArc);
    PPField2dTrajectoryFollowerSequence trajectory3 = new PPField2dTrajectoryFollowerSequence(drive, gridToChargeStation);

    addCommands(
      calibrateElevatorAndWrist,
      placeCube1,
      trajectory1,
      pickUpGamePiece,
      trajectory2,
      placeCube2,
      trajectory3,
      getOnChargeStation,
      chargeStationBalance
    );
    
  }
  
}
