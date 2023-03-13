// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.GoToStateCommand;
import frc.robot.commands.Intake.OpenMandiblesCommand;
import frc.robot.commands.Intake.TimedRunMandiblesCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceHighRowSequence extends SequentialCommandGroup {

  /** Creates a new PlaceGamePieceHighRowSequence. */
  public PlaceHighRowSequence(Elevator elevator, Wrist wrist, Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    TimedRunMandiblesCommand closeMandibles = new TimedRunMandiblesCommand(intake, true, 0.3);
    ToHighRowSequence toHighCube = new ToHighRowSequence(elevator, wrist);
    TimedRunMandiblesCommand openMandibles = new TimedRunMandiblesCommand(intake, false, 0.3);
    StowSequence setTravelState = new StowSequence(elevator, wrist);

    addCommands(
      closeMandibles,
      toHighCube,
      openMandibles,
      setTravelState
    );

  }

}