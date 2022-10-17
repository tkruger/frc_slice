// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import frc.robot.commands.Drivetrain.ResetOdometryCommand;
import frc.robot.commands.Indexer.AutoShoot.IndexerDownSlightCommand;
import frc.robot.commands.Intake.IntakeSchedulableCommand;
import frc.robot.commands.Shooter.AutoShoot.SmartAutoShoot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class pathWeaverAutoRoutine extends SequentialCommandGroup {
  /** Add your docs here. */

  public pathWeaverAutoRoutine(Drivetrain drivetrain, Intake intake, Indexer indexer, Shooter shooter, Limelight limelight, Joystick leftJoystick, Joystick rightJoystick) {

    IntakeSchedulableCommand runIntake = new IntakeSchedulableCommand(intake, true);

    RamseteCommand path1TrajectoryForward = Trajectories.generateRamseteCommand(drivetrain, Paths.getAutoPath(1));
    ParallelDeadlineGroup path1ForwardGroup = new ParallelDeadlineGroup(path1TrajectoryForward, runIntake);

    RamseteCommand path2TrajectoryForward = Trajectories.generateRamseteCommand(drivetrain, Paths.getAutoPath(2));
    ParallelDeadlineGroup path2ForwardGroup = new ParallelDeadlineGroup(path2TrajectoryForward, runIntake);

    RamseteCommand path3TrajectoryForward = Trajectories.generateRamseteCommand(drivetrain, Paths.getAutoPath(3));


    // Add Commands here:
    // e.g. addSequential(new Command1());
    // addSequential(new Command2());
    // these will run in order.
    addCommands(
      new ResetOdometryCommand(drivetrain, new Pose2d()),
      new IndexerDownSlightCommand(indexer),
      path1ForwardGroup,
      new smart2BallShootSequence(indexer, intake, shooter, drivetrain, limelight, leftJoystick, rightJoystick),
      path2ForwardGroup,
      new IndexerDownSlightCommand(indexer),
      path3TrajectoryForward,
      new SmartAutoShoot(limelight, shooter)
    );

    // To run multiple commands at the same time,
    // use addParallel()
    // e.g. addParallel(new Command1());
    // addSequential(new Command2());
    // Command1 and Command2 will run in parallel.

    // A command group will require all of the subsystems that each member
    // would require.
    // e.g. if Command1 requires chassis, and Command2 requires arm,
    // a CommandGroup containing them would require both the chassis and the
    // arm.
  }
}
