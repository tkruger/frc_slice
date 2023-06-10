// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.robot.Constants;
import frc.robot.JoystickFilter;
import frc.robot.subsystems.Wrist;

public class WristControllerRunCommand extends CommandBase {

  private final Wrist m_wrist;
  private final CommandGenericHID m_manipulatorController;

  private final JoystickFilter speedFilter;

/** Creates a new ElevatorRunCommand. */
  public WristControllerRunCommand(Wrist wrist, CommandGenericHID manipulatorController) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(wrist);

    m_wrist = wrist;
    m_manipulatorController = manipulatorController;

    speedFilter = new JoystickFilter(0.05, 0.3, false);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_wrist.enableManualControl();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double wristSpeed = speedFilter.filter(m_manipulatorController.getRawAxis(3));

    if(wristSpeed > 0) {

      if (m_wrist.completelyStowed() || (m_wrist.getAngle() < Constants.Wrist.MIN_ANGLE + 5)) {
        
        m_wrist.spinWrist(0); 

      }
      else {

        m_wrist.spinWrist(wristSpeed);

      }

    }
    else {

      m_wrist.spinWrist(wristSpeed);

    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_wrist.spinWrist(0);
    m_wrist.setWristPosition(m_wrist.getAngle());
    m_wrist.disableManualControl();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return false;
    
  }
  
}
