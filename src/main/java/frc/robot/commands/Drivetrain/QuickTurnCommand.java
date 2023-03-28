package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class QuickTurnCommand extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Drivetrain m_drivetrain;
    
    Rotation2d startRot, endRot;

    private final Timer timeoutTimer;
    
    public QuickTurnCommand(Drivetrain drivetrain) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drivetrain);

        this.m_drivetrain = drivetrain;

        timeoutTimer = new Timer();

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        startRot = m_drivetrain.getRotation2d();
        
        endRot = startRot.plus(new Rotation2d(Math.PI));

        timeoutTimer.reset();
        timeoutTimer.start();

    }

    public void execute() {
        
        m_drivetrain.PIDArcadeDrive(0, -.4);

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_drivetrain.ArcadeDrive(0, 0);
        timeoutTimer.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        //double currentRot = m_drivetrain.getHeading();

        Rotation2d diff = endRot.minus(m_drivetrain.getRotation2d());

        if (Math.abs(diff.getDegrees()) < 5) {
            return true;
        }

        return timeoutTimer.get() > 5;
    }
}
