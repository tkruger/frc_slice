// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.modes.Pathplannerless;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.Intake.TimedRunMandiblesCommand;
import frc.robot.Constants;
import frc.robot.commands.InstantCalibrationCommand;
import frc.robot.commands.Drivetrain.*;
import frc.robot.commands.Wrist.*;
import frc.robot.commands.sequences.ToHighRowSequence;
import frc.robot.subsystems.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreOneLowPieceThenRetrieveCube extends SequentialCommandGroup {
  /** Creates a new ScoreOneGamePieceMobilityThenEngageMode. */
  public ScoreOneLowPieceThenRetrieveCube(Drivetrain drive, Elevator elevator, Wrist wrist, Intake intake, Limelight limelight) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    InstantCalibrationCommand calibrateElevatorAndWrist = new InstantCalibrationCommand(elevator, wrist);
    AutonomousTimedDriveCommand driveFront = new AutonomousTimedDriveCommand(drive, -0.5, 0, 0.3);
    AutonomousTimedDriveCommand driveBack = new AutonomousTimedDriveCommand(drive, 0.5, 0, 0.5);
    AutonomousTimedDriveCommand mobility = new AutonomousTimedDriveCommand(drive, -0.5, 0, 3.25);
    TimedRunMandiblesCommand confirmMandiblesOpen = new TimedRunMandiblesCommand(intake, false, 0.3);
    SetWristPosition setWristGround = new SetWristPosition(wrist, Constants.States.LOW_ROW_GROUND_STATE.wristAngle);
    TimedWristRunCommand runWristUp = new TimedWristRunCommand(wrist, true, 0.3);
    AutonomousTimedDriveCommand pickUpDrive = new AutonomousTimedDriveCommand(drive, -0.4, 0, 1.7);
    SetWristPosition stowWrist = new SetWristPosition(wrist, Constants.States.TRAVEL_STATE.wristAngle);
    TimedRunMandiblesCommand closeMandibles = new TimedRunMandiblesCommand(intake, true, 0.6);
    AutonomousTimedDriveCommand driveToGrid = new AutonomousTimedDriveCommand(drive, 0.6, 0, 3.4);
    AutonomousTimedDriveCommand driveFront2 = new AutonomousTimedDriveCommand(drive, -0.5, 0, 0.5);
    QuickTurnPIDCommand quickTurn = new QuickTurnPIDCommand(drive);
    ToHighRowSequence toHighCube = new ToHighRowSequence(elevator, wrist);

    ParallelCommandGroup groundGroup = new ParallelCommandGroup(confirmMandiblesOpen, setWristGround);
    ParallelCommandGroup stowWhileBacking = new ParallelCommandGroup(driveToGrid, stowWrist);


    //ParallelRaceGroup calibrateElevatorAndWrist = new ParallelCommandGroup(calibrateElevator, resetWristAngle).withTimeout(2);

    addCommands(
      calibrateElevatorAndWrist,
      driveFront,
      driveBack,
      mobility,
      groundGroup,
      runWristUp,
      pickUpDrive,
      closeMandibles,
      stowWhileBacking,
      driveFront2,
      quickTurn
      //toHighCube
    );
  }
}
