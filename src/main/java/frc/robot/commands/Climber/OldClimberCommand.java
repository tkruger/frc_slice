// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** A climber command that uses an climber subsystem. */
@Deprecated
public class OldClimberCommand extends CommandBase {
  // Warning Supression
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  // Instance Variables
  private final Climber m_climber;
  
  double climberSpeed = 0;
  Joystick m_rightJoystick;

  // Constructor
  public OldClimberCommand(Climber climber, Joystick rightJoystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);

    m_climber = climber;
    m_rightJoystick = rightJoystick;
  }

  // Method 1
  @Override
  public void initialize() {
    
  }

  // Method 2
  @Override
  public void execute() {
    boolean climberRetract = m_rightJoystick.getRawButton(7);
    boolean climberExtend = m_rightJoystick.getRawButton(8);    
  
    //Sets climber motors
    if(climberExtend == true)
    {
      climberSpeed = 0.3;
    }
    else if(climberRetract == true)
    {
      climberSpeed = -0.3;
    }
    else
    {
      climberSpeed = 0;
    }

    m_climber.setClimber(climberSpeed);
  }

  // Method 3
  @Override
  public void end(boolean interrupted) {
    m_climber.setClimber(0);
  }

  // Method 4
  @Override
  public boolean isFinished() {
    return false;
  }
}