package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class QuickTurnCommand extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Drivetrain m_drivetrain;
    
    double endRot;
    
    public QuickTurnCommand(Drivetrain drivetrain) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drivetrain);

        this.m_drivetrain = drivetrain;

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {        
        endRot = m_drivetrain.getHeading() + 180;
        if(endRot > 360) {
            endRot -= 360;
        }
        System.out.println(endRot);
    }

    @Override
    public void execute() {
        m_drivetrain.ArcadeDrive(0, -.4);
        System.out.println("drive");
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_drivetrain.ArcadeDrive(0, 0);
        System.out.println("We ended " + interrupted);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        double currentRot = m_drivetrain.getHeading();
        System.out.println("Stop? " + (currentRot > endRot - 20 && currentRot < endRot + 20));
    
        if(currentRot > endRot - 20 && currentRot < endRot + 20) {
            return true;
        }

        return false;
    }
}
