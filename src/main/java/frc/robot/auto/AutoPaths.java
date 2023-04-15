package frc.robot.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class AutoPaths {

    public Trajectory trajectory;

    public SwerveControllerCommand generateSwerveControllerCommand(Drivetrain drive, Trajectory trajectory) {

        return new SwerveControllerCommand(
            trajectory,
            drive::getPose,
            Constants.kDrivetrain.kSwerveKinematics,
                new PIDController(Constants.kAutonomous.kPXController, 0, 0),
                new PIDController(Constants.kAutonomous.kPYController, 0, 0),
                new ProfiledPIDController(
                    Constants.kAutonomous.kPThetaController,
                    0,
                    0,
                    Constants.kAutonomous.kThetaControllerConstraints),
            drive::getAutoTrajectoryRotation,
            //SwerveControllerCommand passes output module states to the callback
            drive::setModuleStates,
            drive);

    }

}
