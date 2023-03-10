// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.modes;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.commands.Elevator.CalibrateElevatorCommand;
import frc.robot.commands.Wrist.ResetAngleCommand;
import frc.robot.commands.sequences.PlaceConeHighRowSequence;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreOneConeHighRowMode extends SequentialCommandGroup {
  /** Creates a new ScoreOneConeHighRow. */
  public ScoreOneConeHighRowMode(Elevator elevator, Wrist wrist, Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    CalibrateElevatorCommand calibrateElevator = new CalibrateElevatorCommand(elevator);
    ResetAngleCommand resetWristAngle = new ResetAngleCommand(wrist);
    PlaceConeHighRowSequence placeCone = new PlaceConeHighRowSequence(elevator, wrist, intake);

    ParallelCommandGroup calibrateElevatorAndWrist = new ParallelCommandGroup(calibrateElevator, resetWristAngle);

    addCommands(
      calibrateElevatorAndWrist,
      placeCone
    );

  }

}
