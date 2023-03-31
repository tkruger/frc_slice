// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Wrist;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Wrist;

public class TimedWristRunCommand extends CommandBase {

  private final Wrist m_wrist;
  private final boolean m_runUpwards;

  private final Timer m_timer;
  private final double m_time;

/** Creates a new ElevatorRunCommand. */
  public TimedWristRunCommand(Wrist wrist, boolean runUpwards, double time) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(wrist);

    m_wrist = wrist;
    m_runUpwards = runUpwards;

    m_timer = new Timer();
    m_time = time;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_wrist.enableManualControl();

    m_timer.reset();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(m_runUpwards) {

      if (m_wrist.completelyStowed() || (m_wrist.getAngle() < Constants.Wrist.MIN_ANGLE + 5)) {
        m_wrist.spinWrist(0); 
      } else {
        m_wrist.spinWrist(-Constants.Wrist.RUN_UP_SPEED);
      }

    }
    else {

      if (m_wrist.completelyStowed() || (m_wrist.getAngle() < Constants.Wrist.MIN_ANGLE + 5)) {
        m_wrist.spinWrist(0); 
      } else {
        m_wrist.spinWrist(Constants.Wrist.RUN_DOWN_SPEED);
      }

    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_wrist.spinWrist(0);
    m_wrist.disableManualControl();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return m_timer.get() > m_time;
    
  }
  
}
