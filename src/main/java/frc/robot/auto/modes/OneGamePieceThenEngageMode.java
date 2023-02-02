// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.modes;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.auto.AutoSelector;
import frc.robot.auto.paths.GridToChargeStationPath;
import frc.robot.auto.paths.PickUpOneGamePiecePath;
import frc.robot.auto.paths.PlaceOneGamePiecePath;
import frc.robot.auto.sequences.TrajectoryFollowerSequence;
import frc.robot.commands.Drivetrain.ChargeStationBalancePIDCommand;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OneGamePieceThenEngageMode extends SequentialCommandGroup {
  /** Creates a new OneGamePieceThenEngageMode. */
  public OneGamePieceThenEngageMode(AutoSelector.StartingPosition startPosition, Drivetrain drive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    PickUpOneGamePiecePath pickUpOneGamePiece = new PickUpOneGamePiecePath(startPosition, drive);
    PlaceOneGamePiecePath placeOneGamePiece = new PlaceOneGamePiecePath(startPosition, drive);
    GridToChargeStationPath gridToChargeStation = new GridToChargeStationPath(startPosition, drive);
    ChargeStationBalancePIDCommand chargeStationBalance = new ChargeStationBalancePIDCommand(drive);

    TrajectoryFollowerSequence trajectory1 = new TrajectoryFollowerSequence(drive, pickUpOneGamePiece, pickUpOneGamePiece.trajectory.getInitialPose());
    TrajectoryFollowerSequence trajectory2 = new TrajectoryFollowerSequence(drive, placeOneGamePiece);
    TrajectoryFollowerSequence trajectory3 = new TrajectoryFollowerSequence(drive, gridToChargeStation);

    addCommands(
      trajectory1,
      trajectory2,
      trajectory3,
      chargeStationBalance
    );
    
  }
  
}
