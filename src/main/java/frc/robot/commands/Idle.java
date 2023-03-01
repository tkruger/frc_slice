package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Idle extends CommandBase {

    /**
     * Creates a new IdleCommand. Use this command for inactive subsystems.
     * 
     * @param subsystem The desired inactive subsystem to use this command.
    */
    public Idle(Subsystem subsystem) {

        addRequirements(subsystem);

    }

}
