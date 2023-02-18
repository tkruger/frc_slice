// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.Constants;
import frc.robot.commands.Intake.CloseMandibles;
import frc.robot.commands.Wrist.ResetAngleCommand;
import frc.robot.commands.Wrist.SetWristPosition;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PickUpGamePieceSequence extends SequentialCommandGroup {
  /** Creates a new PickUpGamePieceSequence. */
  public PickUpGamePieceSequence(Elevator elevator, Wrist wrist, Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    SetWristPosition setWristPickUpPosition = new SetWristPosition(wrist, Constants.wrist_GAME_PIECE_PICK_UP_ANGLE);
    CloseMandibles closeMandibles = new CloseMandibles(intake);
    ResetAngleCommand resetWrist = new ResetAngleCommand(wrist);

    addCommands(
      setWristPickUpPosition,
      closeMandibles,
      resetWrist
    );

  }

}
