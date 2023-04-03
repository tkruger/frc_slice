package frc.robot.auto;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrivetrain;

public class AutoPaths{

    public Trajectory trajectory;

    //This method uses kPDriveVel as the thetaController parameter for ProfiledPIDController() as a placholder for now
    public SwerveControllerCommand generateSwerveControllerCommand(SwerveDrivetrain drive, Trajectory trajectory) {

        return new SwerveControllerCommand(
            trajectory,
            drive::getPose,
            Constants.Drivetrain.kSwerveKinematics,
            new HolonomicDriveController(
                new PIDController(Constants.Autonomous.kPDriveVel, 0, 0),
                new PIDController(Constants.Autonomous.kPDriveVel, 0, 0),
                new ProfiledPIDController(
                    Constants.Autonomous.kPDriveVel,
                    0,
                    0,
                    new Constraints(Constants.Drivetrain.kMaxVelocityMetersPerSecond, Constants.Drivetrain.kMaxAccelerationMetersPerSecondSquared))),
            drive::getAutoTrajectoryRotation,
            //SwerveControllerCommand passes output module states to the callback
            drive::autoOutputModuleStates,
            drive);

    }

}
