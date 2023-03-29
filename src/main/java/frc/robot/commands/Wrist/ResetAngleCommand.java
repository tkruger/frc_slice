// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Wrist;

public class ResetAngleCommand extends CommandBase {
  private final Wrist m_wrist;

  /** Creates a new ResetAngleCommand. */
  public ResetAngleCommand(Wrist wrist) {
    m_wrist = wrist;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_wrist.enableManualControl();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_wrist.spinWrist(-Constants.Wrist.RESET_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_wrist.spinWrist(0);
    m_wrist.setEncoder(Constants.Wrist.MIN_ANGLE);
    m_wrist.disableManualControl();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_wrist.completelyStowed();
  }
}
