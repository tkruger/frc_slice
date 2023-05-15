// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.NodeSelector;
import frc.robot.subsystems.Drivetrain;

public class NodeSequenceCommand extends CommandBase {

  private final NodeSelector m_nodeSelector;
  private final Drivetrain m_drivetrain;
  private SequentialCommandGroup nodeSequence;

  /** Creates a new NodeSequenceCommand. */
  public NodeSequenceCommand(NodeSelector nodeSelector, Drivetrain drivetrain) {
    m_nodeSelector = nodeSelector;
    m_drivetrain = drivetrain;

    // Use addRequirements() here to declare subsystem dependencies.
    //addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    nodeSequence = m_nodeSelector.getNodeSequence();
    nodeSequence.schedule();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    nodeSequence.cancel();
    //m_drivetrain.disablePreventVisionImplementation();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return false;

  }

}
