// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.GoToStateCommand;
import frc.robot.commands.Intake.AutoMandiblesCommand;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PickUpGamePieceDoubleSubstationSequence extends SequentialCommandGroup {
  /** Creates a new PickUpGamePieceDoubleSubstationSequence. */
  public PickUpGamePieceDoubleSubstationSequence(Elevator elevator, Wrist wrist, Intake intake, ColorSensor colorSensor) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    GoToStateCommand setDoubleSubstationPickUpState = new GoToStateCommand(elevator, wrist, Constants.State.DOUBLE_SUBSTATION_STATE);
    AutoMandiblesCommand closeMandibles = new AutoMandiblesCommand(intake, colorSensor);
    GoToStateCommand setTravelState = new GoToStateCommand(elevator, wrist, Constants.State.TRAVEL_STATE);

    addCommands(
      setDoubleSubstationPickUpState,
      closeMandibles,
      setTravelState
    );

  }
  
}