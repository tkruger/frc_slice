// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.sequences;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.auto.AutoPaths;
import frc.robot.commands.Drivetrain.ResetOdometryCommand;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TrajectoryFollowerSequence extends SequentialCommandGroup {

  /** Creates a new TrajectoryFollowerSequence without reseting the position of the robot. */
  public TrajectoryFollowerSequence(Drivetrain drive, AutoPaths autoPath) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    RamseteCommand ramseteCommand = autoPath.generateRamseteCommand(autoPath.trajectory, drive);
    InstantCommand stopDriveCommand = new InstantCommand(drive::stopDrive, drive);

    addCommands(
      ramseteCommand,
      stopDriveCommand);
      
  }

  /** Creates a new TrajectoryFollowerSequence, reseting the position of the robot at the beginning of the sequence. */
  public TrajectoryFollowerSequence(Drivetrain drive, AutoPaths autoPath, Pose2d position) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    ResetOdometryCommand resetOdometryCommand = new ResetOdometryCommand(drive, position);
    RamseteCommand ramseteCommand = autoPath.generateRamseteCommand(autoPath.trajectory, drive);
    InstantCommand stopDriveCommand = new InstantCommand(drive::stopDrive, drive);

    addCommands(
      resetOdometryCommand,
      ramseteCommand,
      stopDriveCommand);
      
  }

}
