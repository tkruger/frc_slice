// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.commands;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.SwerveDrivetrain;

public class PrepareAutoRotationsCommand extends CommandBase {

  private final SwerveDrivetrain m_drivetrain;
  private final Trajectory m_trajectory;

  /** Creates a new GetTrajectoryRotationCommand. */
  public PrepareAutoRotationsCommand(SwerveDrivetrain drivetrain, Trajectory trajectory) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);

    m_drivetrain = drivetrain;
    m_trajectory = trajectory;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_drivetrain.startAutoTrajectoryTimer();
    m_drivetrain.setCurrentAutoTrajectory(m_trajectory);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return true;

  }

}