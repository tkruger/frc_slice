// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Indexer;

import frc.robot.RobotContainer;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class IndexerCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Indexer m_indexer;
  private final Joystick leftJoystick, rightJoystick;

  double Speed = 0;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public IndexerCommand(Indexer indexer, Joystick leftJoystick, Joystick rightJoystick) {
    m_indexer = indexer;
    
    this.leftJoystick = leftJoystick;
    this.rightJoystick = rightJoystick;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    boolean indexerForwardFast = leftJoystick.getRawButton(5);
    boolean indexerForwardSlow = leftJoystick.getRawButton(3);
    boolean indexerBackwardFast = rightJoystick.getRawButton(5);    
    boolean indexerBackwardSlow = rightJoystick.getRawButton(3); 
    
    //Sets Indexer motor
    if(indexerForwardFast == true) {
      Speed = 0.6;
    }
    else if(indexerBackwardFast == true) {
      Speed = -0.6;
    }
    else if(indexerForwardSlow == true) {
      Speed = 0.3;
    }
    else if(indexerBackwardSlow == true) {
      Speed = -0.3;
    }
    else {
      Speed = 0;
    }
    
    RobotContainer.m_indexer.SetIndexer(Speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_indexer.SetIndexer(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
