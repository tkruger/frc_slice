// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.LambdaCommand;
import frc.robot.commands.Drivetrain.AutonomousTimedDriveCommand;
import frc.robot.commands.Drivetrain.sequences.Field2dTrajectoryFollowerSequence;
//import frc.robot.commands.Drivetrain.sequences.TrajectoryFollowerSequence;
//import frc.robot.commands.Drivetrain.AutonomousDistanceDriveCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DoubleSubstationAlignAndPositionSequence extends SequentialCommandGroup {
  /** Creates a new DoubleSubstationAlignThenPickUpPieceSequence. */
  public DoubleSubstationAlignAndPositionSequence(Drivetrain drive, Elevator elevator, Wrist wrist) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    LambdaCommand doubleSubstationAlign = new LambdaCommand(() -> new Field2dTrajectoryFollowerSequence(drive, Limelight.generateDoubleSubstationTrajectory(drive.getPose())));
    AutonomousTimedDriveCommand moveForward = new AutonomousTimedDriveCommand(drive, 0.5, 0, 0.55);
    ToDoubleSubstationSequence toDoubleSubstation = new ToDoubleSubstationSequence(elevator, wrist);

    addCommands(
      doubleSubstationAlign,
      moveForward,
      toDoubleSubstation
    );

  }

}
