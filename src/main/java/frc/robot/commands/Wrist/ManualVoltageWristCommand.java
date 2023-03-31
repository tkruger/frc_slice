// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Wrist;

public class ManualVoltageWristCommand extends CommandBase {
  private final Wrist m_wrist;
  private final double m_voltage;
  /** Creates a new ManualWristCommand. */
  public ManualVoltageWristCommand(Wrist wrist, double voltage) {
    m_wrist = wrist;
    m_voltage = voltage;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_wrist.enableManualControl();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if ((m_wrist.completelyStowed() && m_voltage > 0) || (m_wrist.getAngle() < Constants.Wrist.MIN_ANGLE && m_voltage < 0)) {
      m_wrist.setWristVoltage(0);
    } else {
      m_wrist.setWristVoltage(m_voltage);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_wrist.spinWrist(0);
    m_wrist.disableManualControl();
    m_wrist.setWristPosition(m_wrist.getAngle());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
