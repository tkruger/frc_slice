package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class QuickTurnCommand extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Drivetrain m_drivetrain;
    
    double startRot, endRot;
    
    public QuickTurnCommand(Drivetrain drivetrain) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drivetrain);

        this.m_drivetrain = drivetrain;

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        startRot = m_drivetrain.getHeading();
        
        endRot = startRot + 180;
        if(endRot > 360) {
            endRot -= 360;
        }
    }

    public void execute() {
        m_drivetrain.ArcadeDrive(0, -.8);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_drivetrain.ArcadeDrive(0, 0);
        
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        double currentRot = m_drivetrain.getHeading();

        if(currentRot > endRot - 55 && currentRot < endRot + 55) {
            return true;
        }

        return false;
    }
}
