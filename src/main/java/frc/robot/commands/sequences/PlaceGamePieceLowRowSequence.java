// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.Constants;
import frc.robot.commands.GoToStateCommand;
import frc.robot.commands.Intake.OpenMandiblesCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceGamePieceLowRowSequence extends SequentialCommandGroup {
  /** Creates a new PlaceGamePieceLowRowSequence. */
  public PlaceGamePieceLowRowSequence(Elevator elevator, Wrist wrist, Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    GoToStateCommand setLowRowState = new GoToStateCommand(elevator, wrist, Constants.States.LOW_ROW_GROUND_STATE);
    OpenMandiblesCommand openMandibles = new OpenMandiblesCommand(intake);
    GoToStateCommand setTravelState = new GoToStateCommand(elevator, wrist, Constants.States.TRAVEL_STATE);

    addCommands(
      setLowRowState,
      openMandibles,
      setTravelState
    );

  }

}