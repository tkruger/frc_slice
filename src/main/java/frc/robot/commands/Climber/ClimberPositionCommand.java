// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import frc.robot.Constants;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** A climber command that uses an climber subsystem. */
public class ClimberPositionCommand extends CommandBase {
  // Warning Supression
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  // Instance Variables
  private final Climber m_climber;
  
  double climberSpeed = 0;
  double targetPosition;

  boolean leftFinished = false;
  boolean rightFinished = false;
  
  // Constructor
  public ClimberPositionCommand(Climber climber, double targetPosition) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);

    m_climber = climber;
    this.targetPosition = targetPosition;
  }

  // Method 1
  @Override
  public void initialize() {
    //Sets climber motors
    climberSpeed = Constants.climber_SPEED;
  }

  // Method 2
  @Override
  public void execute() {  

    if(Math.abs(Math.abs(targetPosition) - Math.abs(m_climber.leftClimbPosition)) < 1) {
      leftFinished = true;
      m_climber.setLeftClimber(0);
    } else if(targetPosition > m_climber.leftClimbPosition) {
      m_climber.setLeftClimber(climberSpeed);
    } else if(targetPosition < m_climber.leftClimbPosition) {
      m_climber.setLeftClimber(-climberSpeed);
    }

    if(Math.abs(Math.abs(targetPosition) - Math.abs(m_climber.rightClimbPosition)) < 1) {
      rightFinished = true;
      m_climber.setRightClimber(0);
    } else if(targetPosition > m_climber.rightClimbPosition) {
      m_climber.setRightClimber(climberSpeed);
    } else if(targetPosition < m_climber.rightClimbPosition) {
      m_climber.setRightClimber(-climberSpeed);
    }

  }

  // Method 3
  @Override
  public void end(boolean interrupted) {
    m_climber.setClimber(0);
  }

  // Method 4
  @Override
  public boolean isFinished() {
    return (rightFinished && leftFinished);
  }
}