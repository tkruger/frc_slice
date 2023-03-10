// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LEDs;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDs;

public class FlashColorCommand extends CommandBase {
  private final LEDs m_LEDs;
  private final Color color;
  private final double onTime, offTime, totalTime;
  private final Timer timer;
  private boolean on;
  /** Creates a new FlashColorCommand. */
  public FlashColorCommand(LEDs LEDs, Color color, double onTime, double offTime) {
    m_LEDs = LEDs;
    this.color = color;
    this.onTime = onTime;
    this.offTime = offTime;
    totalTime = onTime + offTime;

    timer = new Timer();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_LEDs);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_LEDs.setAll(Color.kBlack);
    timer.reset();
    timer.start();
    on = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if ((timer.get() % totalTime) > offTime && !on) {
      m_LEDs.setAll(color);
      on = true;
    }else if ((timer.get() % totalTime) <= offTime && on) {
      m_LEDs.setAll(Color.kBlack);
      on = false;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_LEDs.setAll(Color.kBlack);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}