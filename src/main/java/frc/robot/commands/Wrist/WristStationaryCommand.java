// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Wrist;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Wrist;

public class WristStationaryCommand extends CommandBase {

  private final Wrist m_wrist;
  private final Timer timer;
  private boolean run = false;

  /** Creates a new WristStationaryCommand. */
  public WristStationaryCommand(Wrist wrist) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(wrist);

    m_wrist = wrist;

    timer = new Timer();

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    run = false;
    timer.reset();
    timer.start();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.get() < 1) {
      m_wrist.spinWrist(0);
    }else if (timer.get() > 1 && !run) {
      run = true;
      m_wrist.setPID(Constants.Wrist.KP, Constants.Wrist.KI, Constants.Wrist.KD);
      m_wrist.setWristPosition(m_wrist.getAngle());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return false;

  }

}
