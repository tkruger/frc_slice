// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.modes;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.auto.AutoSelector;
import frc.robot.auto.paths.GridToGamePiecePath;
import frc.robot.auto.sequences.TrajectoryFollowerSequence;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestAutoMode extends SequentialCommandGroup {
  /** Creates a new TestAutoMode. */
  public TestAutoMode(AutoSelector.StartingPosition startingPosition, Drivetrain drive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    GridToGamePiecePath gridToGamePiece = new GridToGamePiecePath(startingPosition);

    TrajectoryFollowerSequence trajectory = new TrajectoryFollowerSequence(drive, gridToGamePiece, gridToGamePiece.trajectory.getInitialPose());

    addCommands(
      trajectory
    );

  }

}
