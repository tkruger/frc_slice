// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.commands.GoToStateCommand;
import frc.robot.commands.Wrist.SetWristPosition;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class StowSequence extends SequentialCommandGroup {
  /** Creates a new StowSequence. */
  public StowSequence(Elevator elevator, Wrist wrist) {

    SetWristPosition transitionState = new SetWristPosition(wrist, -60);
    GoToStateCommand stowState = new GoToStateCommand(elevator, wrist, Constants.States.TRAVEL_STATE);
    
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      transitionState,
      stowState
    );
  }
}
