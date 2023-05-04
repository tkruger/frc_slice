// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.modes.Pathplannerless;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands.Drivetrain.ChargeStation.BoardChargeStationCommand;
import frc.robot.commands.InstantCalibrationCommand;
import frc.robot.commands.Drivetrain.AutonomousTimedDriveCommand;
import frc.robot.commands.Drivetrain.QuickTurnSequence;
import frc.robot.commands.Drivetrain.ChargeStation.ChargeStationBalancePIDCommand;
import frc.robot.commands.sequences.PlaceHighRowSequence;
import frc.robot.subsystems.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreOneGamePieceMobilityThenEngageMode extends SequentialCommandGroup {
  /** Creates a new ScoreOneGamePieceMobilityThenEngageMode. */
  public ScoreOneGamePieceMobilityThenEngageMode(Drivetrain drive, Elevator elevator, Wrist wrist, Intake intake, LEDs leds) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    // CalibrateElevatorCommand calibrateElevator = new CalibrateElevatorCommand(elevator);
    // ResetAngleCommand resetWristAngle = new ResetAngleCommand(wrist);
    InstantCalibrationCommand calibrateElevatorAndWrist = new InstantCalibrationCommand(elevator, wrist);
    PlaceHighRowSequence placePiece = new PlaceHighRowSequence(elevator, wrist, intake);
    AutonomousTimedDriveCommand driveBack = new AutonomousTimedDriveCommand(drive, 0.5, 0, 0.3);
    QuickTurnSequence quickTurn = new QuickTurnSequence(drive);
    QuickTurnSequence quickTurn2 = new QuickTurnSequence(drive);
    AutonomousTimedDriveCommand mobility = new AutonomousTimedDriveCommand(drive, -0.5, 0, 5);
    BoardChargeStationCommand getOnChargeStation = new BoardChargeStationCommand(drive);
    ChargeStationBalancePIDCommand chargeStationBalance = new ChargeStationBalancePIDCommand(drive, leds);

    //ParallelRaceGroup calibrateElevatorAndWrist = new ParallelCommandGroup(calibrateElevator, resetWristAngle).withTimeout(2);

    addCommands(
      calibrateElevatorAndWrist,
      placePiece,
      driveBack,
      quickTurn,
      mobility,
      quickTurn2,
      getOnChargeStation,
      chargeStationBalance
    );
  }
}
