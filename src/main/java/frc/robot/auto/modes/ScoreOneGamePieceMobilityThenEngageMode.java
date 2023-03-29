// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.modes;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands.Drivetrain.ChargeStation.AutonomousAngleDriveCommand;
import frc.robot.commands.Drivetrain.AutonomousTimedDriveCommand;
import frc.robot.commands.Drivetrain.ChargeStation.ChargeStationBalancePIDCommand;
import frc.robot.commands.Elevator.CalibrateElevatorCommand;
import frc.robot.commands.Wrist.ResetAngleCommand;
import frc.robot.commands.sequences.PlaceHighRowSequence;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreOneGamePieceMobilityThenEngageMode extends SequentialCommandGroup {
  /** Creates a new ScoreOneGamePieceMobilityThenEngageMode. */
  public ScoreOneGamePieceMobilityThenEngageMode(Drivetrain drive, Elevator elevator, Wrist wrist, Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    CalibrateElevatorCommand calibrateElevator = new CalibrateElevatorCommand(elevator);
    ResetAngleCommand resetWristAngle = new ResetAngleCommand(wrist);
    PlaceHighRowSequence placePiece = new PlaceHighRowSequence(elevator, wrist, intake);
    AutonomousTimedDriveCommand mobility = new AutonomousTimedDriveCommand(drive, 0.25, 0, 6);
    AutonomousAngleDriveCommand driveToChargeStation = new AutonomousAngleDriveCommand(drive, -0.25);
    AutonomousTimedDriveCommand continueDrive = new AutonomousTimedDriveCommand(drive, -0.8, 0, 0.3);
    ChargeStationBalancePIDCommand chargeStationBalance = new ChargeStationBalancePIDCommand(drive);

    ParallelRaceGroup calibrateElevatorAndWrist = new ParallelCommandGroup(calibrateElevator, resetWristAngle).withTimeout(2);

    addCommands(
      calibrateElevatorAndWrist,
      placePiece,
      mobility,
      driveToChargeStation,
      continueDrive,
      chargeStationBalance
    );
  }
}