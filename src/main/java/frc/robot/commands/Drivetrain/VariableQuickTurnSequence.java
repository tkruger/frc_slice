package frc.robot.commands.Drivetrain;

import frc.robot.subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class VariableQuickTurnSequence extends SequentialCommandGroup {
    public VariableQuickTurnSequence(Drivetrain drive, double turnAngle) {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
    
        addCommands(
          new VariableQuickTurnPIDCommand(drive, turnAngle),
          new WaitCommand(0.25)
        );
          
    }
}
