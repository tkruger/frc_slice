package frc.robot.auto;

import java.util.function.Supplier;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

import frc.robot.Constants;
import frc.robot.commands.Drivetrain.Lambda.LambdaRamseteCommand;
import frc.robot.subsystems.Drivetrain;

public class AutoPaths {

    public PathPlannerTrajectory trajectory;

    public static PPRamseteCommand generatePPRamseteCommand(PathPlannerTrajectory trajectory, Drivetrain drive) {

        return new PPRamseteCommand(
            trajectory,
            drive::getPose,
            new RamseteController(Constants.Autonomous.kRamseteB, Constants.Autonomous.kRamseteZeta),
            new SimpleMotorFeedforward(
                Constants.Autonomous.ksVolts,
                Constants.Autonomous.kvVoltsSecondsPerMeter,
                Constants.Autonomous.kaVoltsSecondsSquaredPerMeter),
            Constants.Drivetrain.kDriveKinematics,
            drive::getWheelSpeeds,
            new PIDController(Constants.Autonomous.kPDriveVel, 0, 0),
            new PIDController(Constants.Autonomous.kPDriveVel, 0, 0),
            // PPRamseteCommand passes volts to the callback
            drive::tankDriveVolts,
            drive);

    }

    public static RamseteCommand generateRamseteCommand(Trajectory trajectory, Drivetrain drive) {

        return new RamseteCommand(
            trajectory,
            drive::getPose,
            new RamseteController(Constants.Autonomous.kRamseteB, Constants.Autonomous.kRamseteZeta),
            new SimpleMotorFeedforward(
                Constants.Autonomous.ksVolts,
                Constants.Autonomous.kvVoltsSecondsPerMeter,
                Constants.Autonomous.kaVoltsSecondsSquaredPerMeter),
            Constants.Drivetrain.kDriveKinematics,
            drive::getWheelSpeeds,
            new PIDController(Constants.Autonomous.kPDriveVel, 0, 0),
            new PIDController(Constants.Autonomous.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            drive::tankDriveVolts,
            drive);

    }

    public static LambdaRamseteCommand generateLambdaRamseteCommand(Supplier<Trajectory> trajectory, Drivetrain drive) {

        return new LambdaRamseteCommand(
            trajectory,
            drive::getPose,
            new RamseteController(Constants.Autonomous.kRamseteB, Constants.Autonomous.kRamseteZeta),
            new SimpleMotorFeedforward(
                Constants.Autonomous.ksVolts,
                Constants.Autonomous.kvVoltsSecondsPerMeter,
                Constants.Autonomous.kaVoltsSecondsSquaredPerMeter),
            Constants.Drivetrain.kDriveKinematics,
            drive::getWheelSpeeds,
            new PIDController(Constants.Autonomous.kPDriveVel, 0, 0),
            new PIDController(Constants.Autonomous.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            drive::tankDriveVolts,
            drive);

    }
    
}
