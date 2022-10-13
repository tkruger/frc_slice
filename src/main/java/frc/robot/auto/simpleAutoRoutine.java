// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drivetrain.ResetOdometryCommand;
import frc.robot.commands.Drivetrain.SimpleAutoDrive;
import frc.robot.commands.Indexer.AutoShoot.TimedIndexerCommand;
import frc.robot.commands.Intake.IntakeCommand;
//import frc.robot.commands.Shooter.AutoShoot.ConstantSpeedShooterCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class simpleAutoRoutine extends SequentialCommandGroup {

  /** Creates a new simpleAutoRoutine. */
  public simpleAutoRoutine(Indexer indexer, Intake intake, Shooter shooter, Drivetrain drivetrain, Limelight limelight, 
  Joystick leftJoystick, Joystick rightJoystick) {

    SimpleAutoDrive forward = new SimpleAutoDrive(drivetrain, 1.5);
    IntakeCommand runIntake = new IntakeCommand(intake, true);
    ParallelDeadlineGroup forwardGroup = new ParallelDeadlineGroup(forward, runIntake);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ResetOdometryCommand(drivetrain, new Pose2d()),
      new TimedIndexerCommand(indexer, -0.5, 0.5),
      forwardGroup,
      new smart2BallShootSequence(indexer, intake, shooter, drivetrain, limelight, leftJoystick, rightJoystick)
    );
  }
}
