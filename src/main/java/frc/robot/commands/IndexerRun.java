// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class IndexerRun extends CommandBase {
  /** Creates a new IndexerRun. */
  public IndexerRun() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_indexer);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    boolean indexerMotorGoingForward = RobotContainer.rightJoystick.getRawButton(2);
    boolean indexerMotorGoingBackward = RobotContainer.rightJoystick.getRawButton(3);

    RobotContainer.m_indexer.runIndexer(indexerMotorGoingForward, indexerMotorGoingBackward);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    RobotContainer.m_indexer.runIndexer(false, false);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return false;
    
  }
}
