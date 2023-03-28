// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.modes;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.auto.AutoSelector;
import frc.robot.auto.paths.GridOutOfCommunityToChargeStationPath;
import frc.robot.commands.Drivetrain.sequences.Field2dTrajectoryFollowerSequence;
import frc.robot.commands.Drivetrain.AutonomousTimedDriveCommand;
import frc.robot.commands.Drivetrain.QuickTurnSequence;
import frc.robot.commands.Drivetrain.ChargeStation.ChargeStationBalancePIDCommand;
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
public class ScoreOneCubeGoOutThenEngageMode extends SequentialCommandGroup {
  /** Creates a new SocreOneCubeGoOutThenEngageMode. */
  public ScoreOneCubeGoOutThenEngageMode(AutoSelector.StartingPosition startPosition, Drivetrain drive, Elevator elevator, Wrist wrist, Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    CalibrateElevatorCommand calibrateElevator = new CalibrateElevatorCommand(elevator);
    ResetAngleCommand resetWristAngle = new ResetAngleCommand(wrist);
    PlaceHighRowSequence placeCube = new PlaceHighRowSequence(elevator, wrist, intake);
    AutonomousTimedDriveCommand driveBack = new AutonomousTimedDriveCommand(drive, 0.5, 0, 0.3);
    QuickTurnSequence quickTurn = new QuickTurnSequence(drive);
    GridOutOfCommunityToChargeStationPath gridOutOfCommunityToChargeStation = new GridOutOfCommunityToChargeStationPath(startPosition);
    ChargeStationBalancePIDCommand chargeStationBalance = new ChargeStationBalancePIDCommand(drive);

    ParallelCommandGroup calibrateElevatorAndWrist = new ParallelCommandGroup(calibrateElevator, resetWristAngle);
    Field2dTrajectoryFollowerSequence trajectory = new Field2dTrajectoryFollowerSequence(drive, gridOutOfCommunityToChargeStation, gridOutOfCommunityToChargeStation.trajectory.getInitialPose());

    addCommands(
      calibrateElevatorAndWrist,
      placeCube,
      driveBack,
      quickTurn,
      trajectory,
      chargeStationBalance
    );

  }

}
