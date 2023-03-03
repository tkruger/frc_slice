// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class SetField2dCommand extends CommandBase {

  private final Drivetrain m_drivetrain;

  private final PathPlannerTrajectory m_trajectory;

 /** Creates a new SetField2dCommand. */
  public SetField2dCommand(PathPlannerTrajectory trajectory, Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);

    m_trajectory = trajectory;
    m_drivetrain = drivetrain;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_drivetrain.setField2d(m_trajectory);
    
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