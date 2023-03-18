// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.GoToStateCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ToDoubleSubstationSequence extends SequentialCommandGroup {
  /** Creates a new ToDoubleSubstationSequence. */
  public ToDoubleSubstationSequence(Elevator elevator, Wrist wrist) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    GoToStateCommand setOutState = new GoToStateCommand(elevator, wrist, Constants.States.TRANSITION_OUT_STATE);
    GoToStateCommand setUpState = new GoToStateCommand(elevator, wrist, Constants.States.TRANSITION_DOUBLE_SUBSTATION_STATE);
    GoToStateCommand setDoubleSubstationState = new GoToStateCommand(elevator, wrist, Constants.States.DOUBLE_SUBSTATION_STATE);
    
    addCommands(
      setOutState,
      setUpState,
      setDoubleSubstationState
    );
  }
}
