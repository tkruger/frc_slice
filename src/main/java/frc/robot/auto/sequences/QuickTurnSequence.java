package frc.robot.auto.sequences;

import frc.robot.commands.Drivetrain.*;
import frc.robot.subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class QuickTurnSequence extends SequentialCommandGroup {
    public QuickTurnSequence(Drivetrain drive) {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
    
        addCommands(
          new QuickTurnCommand(drive),
          new WaitCommand(0.1)
        );
          
    }
}
