// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.modes;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.Drivetrain.ChargeStation.BoardChargeStationCommand;
import frc.robot.commands.Drivetrain.ChargeStation.ChargeStationBalanceCommand;
import frc.robot.commands.Drivetrain.AutonomousTimedDriveCommand;
import frc.robot.commands.Drivetrain.ChargeStation.ChargeStationBalancePIDCommand;
import frc.robot.commands.Drivetrain.QuickTurnSequence;
import frc.robot.commands.Elevator.CalibrateElevatorCommand;
import frc.robot.commands.Wrist.ResetAngleCommand;
import frc.robot.commands.sequences.PlaceHighRowSequence;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreOneGamePieceThenEngageMode extends SequentialCommandGroup {
  /** Creates a new ScoreOneGamePieceThenEngageMode. */
  public ScoreOneGamePieceThenEngageMode(Drivetrain drive, Elevator elevator, Wrist wrist, Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    CalibrateElevatorCommand calibrateElevator = new CalibrateElevatorCommand(elevator);
    ResetAngleCommand resetWristAngle = new ResetAngleCommand(wrist);
    PlaceHighRowSequence placePiece = new PlaceHighRowSequence(elevator, wrist, intake);
    AutonomousTimedDriveCommand driveBack = new AutonomousTimedDriveCommand(drive, 0.5, 0, 0.3);
    QuickTurnSequence quickTurn = new QuickTurnSequence(drive);
    BoardChargeStationCommand getOnChargeStation = new BoardChargeStationCommand(drive);
    ChargeStationBalanceCommand chargeStationBalance = new ChargeStationBalanceCommand(drive);

    ParallelRaceGroup calibrateElevatorAndWrist = new ParallelCommandGroup(calibrateElevator, resetWristAngle).withTimeout(1);

    addCommands(
      calibrateElevatorAndWrist,
      placePiece,
      driveBack,
      quickTurn,
      getOnChargeStation,
      new WaitCommand(2),
      chargeStationBalance
    );
  }
}
