// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.modes;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.auto.AutoSelector;
import frc.robot.auto.paths.GridOutOfCommunityToChargeStationPath;
import frc.robot.auto.sequences.Field2dTrajectoryFollowerSequence;
import frc.robot.commands.Drivetrain.ChargeStationBalancePIDCommand;
import frc.robot.commands.sequences.PlaceGamePieceMidRowSequence;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OneCubeGoOutThenEngageMode extends SequentialCommandGroup {
  /** Creates a new OneGamePieceGoOutThenEngageMode. */
  public OneCubeGoOutThenEngageMode(AutoSelector.StartingPosition startPosition, Drivetrain drive, Elevator elevator, Wrist wrist, Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    PlaceGamePieceMidRowSequence placeCube = new PlaceGamePieceMidRowSequence(elevator, wrist, intake);
    GridOutOfCommunityToChargeStationPath gridOutOfCommunityToChargeStation = new GridOutOfCommunityToChargeStationPath(startPosition, drive);
    ChargeStationBalancePIDCommand chargeStationBalance = new ChargeStationBalancePIDCommand(drive);

    Field2dTrajectoryFollowerSequence trajectory = new Field2dTrajectoryFollowerSequence(drive, gridOutOfCommunityToChargeStation, gridOutOfCommunityToChargeStation.trajectory.getInitialPose());

    addCommands(
      placeCube,
      trajectory,
      chargeStationBalance
    );

  }

}
