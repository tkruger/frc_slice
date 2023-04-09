// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain.sequences;

import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.auto.AutoPaths;
import frc.robot.commands.Drivetrain.ResetOdometryCommand;
import frc.robot.commands.Drivetrain.SetField2dCommand;
import frc.robot.commands.Drivetrain.SetPreventVisionImplementationCommand;
import frc.robot.subsystems.Drivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PPField2dTrajectoryFollowerSequence extends SequentialCommandGroup {

  /** Creates a new PPField2dTrajectoryFollowerSequence without reseting the position of the robot. */
  public PPField2dTrajectoryFollowerSequence(Drivetrain drive, AutoPaths autoPath) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    SetPreventVisionImplementationCommand preventVisionImplementation = new SetPreventVisionImplementationCommand(drive, true);
    SetField2dCommand setField2dCommand = new SetField2dCommand(autoPath.trajectory, drive);
    PPRamseteCommand ppRamseteCommand = AutoPaths.generatePPRamseteCommand(autoPath.trajectory, drive);
    InstantCommand stopDriveCommand = new InstantCommand(drive::stopDrive, drive);
    SetPreventVisionImplementationCommand allowVisionImplementation = new SetPreventVisionImplementationCommand(drive, false);

    addCommands(
      preventVisionImplementation,
      setField2dCommand,
      ppRamseteCommand,
      stopDriveCommand,
      allowVisionImplementation);
      
  }

  /** Creates a new PPField2dTrajectoryFollowerSequence, reseting the position of the robot at the beginning of the sequence. */
  public PPField2dTrajectoryFollowerSequence(Drivetrain drive, AutoPaths autoPath, Pose2d position) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    SetPreventVisionImplementationCommand preventVisionImplementation = new SetPreventVisionImplementationCommand(drive, true);
    ResetOdometryCommand resetOdometry = new ResetOdometryCommand(drive, position);
    SetField2dCommand setField2dCommand = new SetField2dCommand(autoPath.trajectory, drive);
    PPRamseteCommand ppRamseteCommand = AutoPaths.generatePPRamseteCommand(autoPath.trajectory, drive);
    InstantCommand stopDriveCommand = new InstantCommand(drive::stopDrive, drive);
    SetPreventVisionImplementationCommand allowVisionImplementation = new SetPreventVisionImplementationCommand(drive, false);

    addCommands(
      preventVisionImplementation,
      resetOdometry,
      setField2dCommand,
      ppRamseteCommand,
      stopDriveCommand,
      allowVisionImplementation);
      
  }

}
