// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.modes.Pathplannerless;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Drivetrain.ChargeStation.AutonomousAngleDriveCommand;
import frc.robot.commands.InstantCalibrationCommand;
import frc.robot.commands.Drivetrain.AutonomousTimedDriveCommand;
import frc.robot.commands.Drivetrain.QuickTurnSequence;
import frc.robot.commands.Drivetrain.ChargeStation.ChargeStationBalancePIDCommand;
import frc.robot.commands.Limelight.LimelightXAlignmentCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreOneLowPieceMobilityThenEngageMode extends SequentialCommandGroup {
  /** Creates a new ScoreOneGamePieceMobilityThenEngageMode. */
  public ScoreOneLowPieceMobilityThenEngageMode(Drivetrain drive, Elevator elevator, Wrist wrist, Intake intake, Limelight limelight) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    // CalibrateElevatorCommand calibrateElevator = new CalibrateElevatorCommand(elevator);
    // ResetAngleCommand resetWristAngle = new ResetAngleCommand(wrist);
    InstantCalibrationCommand calibrateElevatorAndWrist = new InstantCalibrationCommand(elevator, wrist);
    AutonomousTimedDriveCommand driveFront = new AutonomousTimedDriveCommand(drive, -0.5, 0, 0.3);
    AutonomousTimedDriveCommand driveBack = new AutonomousTimedDriveCommand(drive, 0.5, 0, 0.5);
    QuickTurnSequence halfQuickTurn = new QuickTurnSequence(drive);
    AutonomousTimedDriveCommand mobility = new AutonomousTimedDriveCommand(drive, -0.6, 0, 3.55);
    AutonomousAngleDriveCommand getOnChargeStation = new AutonomousAngleDriveCommand(drive, -0.5);
    AutonomousTimedDriveCommand continueDrive = new AutonomousTimedDriveCommand(drive, -0.7, 0, 0.6);
    ChargeStationBalancePIDCommand chargeStationBalance = new ChargeStationBalancePIDCommand(drive);
    LimelightXAlignmentCommand alignWithApriltag = new LimelightXAlignmentCommand(limelight, drive, 6);

    //ParallelRaceGroup calibrateElevatorAndWrist = new ParallelCommandGroup(calibrateElevator, resetWristAngle).withTimeout(2);

    addCommands(
      calibrateElevatorAndWrist,
      driveFront,
      driveBack,
      mobility,
      halfQuickTurn,
      getOnChargeStation,
      new WaitCommand(0.2),
      continueDrive,
      chargeStationBalance
    );
  }
}
