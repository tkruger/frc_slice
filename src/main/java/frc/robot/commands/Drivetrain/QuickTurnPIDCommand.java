package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class QuickTurnPIDCommand extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Drivetrain m_drivetrain;
    
    Rotation2d startRot, endRot;

    PIDController positionalPID;

    private final Timer timeoutTimer;
    
    public QuickTurnPIDCommand(Drivetrain drivetrain) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drivetrain);

        this.m_drivetrain = drivetrain;

        positionalPID = new PIDController(0.017/2, 0.0005, 0.00000);
        m_drivetrain.setPIDF(.06, .000002, .12, .62);

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
        
        m_drivetrain.PIDArcadeDrive(0, -Math.min(positionalPID.calculate(m_drivetrain.getRotation2d().getDegrees(), endRot.getDegrees()), 3));

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

        if (Math.abs(diff.getDegrees()) < 10) {
            return true;
        }

        // Timeout after 3 seconds
        if(timeoutTimer.get() > 3) {
            return true;
        }

        return false;
    }
}
