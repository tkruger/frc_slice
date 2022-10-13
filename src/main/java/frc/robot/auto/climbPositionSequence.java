// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Climber.*;
import frc.robot.commands.Pneumatics.*;
import frc.robot.subsystems.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class climbPositionSequence extends SequentialCommandGroup {
  /** Creates a new alignlessShootSequence. */
  public climbPositionSequence(Climber climber, Pneumatics pneumatics, boolean pneumaticsIn, double climbTarget) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    if(pneumaticsIn) {
      addCommands(
        new PneumaticsInCommand(pneumatics),
        new ClimberPositionCommand(climber, climbTarget)
      );
    } else {
      addCommands(
        new PneumaticsOutCommand(pneumatics),
        new ClimberPositionCommand(climber, climbTarget)
      );
    }
    
  }
}
