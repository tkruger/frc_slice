// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Wrist;

public class SetWristPosition extends CommandBase {
  private final Wrist m_wrist;
  private final double m_angle;
  private final Timer m_timer;
  /** Creates a new SetWristPosition. */
  public SetWristPosition(Wrist wrist, double angle) {
    m_wrist = wrist;
    m_angle = MathUtil.clamp(angle, Constants.Wrist.MIN_ANGLE, Constants.Wrist.MAX_ANGLE);

    m_timer = new Timer();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_wrist.setPID(Constants.Wrist.KP, Constants.Wrist.KI, Constants.Wrist.KD);
    m_wrist.setWristPosition(m_angle);

    m_timer.reset();
    m_timer.start();
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
    return m_wrist.atTargetPosition();
  }
}
