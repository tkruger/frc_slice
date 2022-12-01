// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import frc.robot.commands.Drivetrain.TrajectoryFollowerSequence;
import frc.robot.subsystems.SwerveDrivetrain;
//import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class trajectoryFollowerAutoRoutine extends SequentialCommandGroup {

  /** Creates a new pathWeaverAutoRoutine. */
  public trajectoryFollowerAutoRoutine(SwerveDrivetrain drivetrain, Joystick leftJoystick, Joystick rightJoystick) {

    //Trajectories m_trajectories = new Trajectories();

    TrajectoryFollowerSequence path1TrajectoryForward = new TrajectoryFollowerSequence(drivetrain, Paths.getAutoPath().get(0), Paths.getAutoPath().get(0).getInitialPose());
    UpdateField2dCommand path1Field2d = new UpdateField2dCommand(1);
    ParallelDeadlineGroup path1ForwardGroup = new ParallelDeadlineGroup(path1TrajectoryForward, path1Field2d);

    TrajectoryFollowerSequence path2TrajectoryForward = new TrajectoryFollowerSequence(drivetrain, Paths.getAutoPath().get(1));
    UpdateField2dCommand path2Field2d = new UpdateField2dCommand(2);
    ParallelDeadlineGroup path2ForwardGroup = new ParallelDeadlineGroup(path2TrajectoryForward, path2Field2d);

    TrajectoryFollowerSequence path3TrajectoryForward = new TrajectoryFollowerSequence(drivetrain, Paths.getAutoPath().get(2));
    UpdateField2dCommand path3Field2d = new UpdateField2dCommand(3);
    ParallelDeadlineGroup path3ForwardGroup = new ParallelDeadlineGroup(path3TrajectoryForward, path3Field2d);


    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());S
    addCommands(
      path1ForwardGroup,
      path2ForwardGroup,
      path3ForwardGroup);

  }
  
}
