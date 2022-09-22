// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Limelight;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;

public class LimelightIdleCommand extends CommandBase {
  /** Creates a new LimelightRun. */

  private final Limelight m_limelight;

  public LimelightIdleCommand(Limelight limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(limelight);

    this.m_limelight = limelight;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_limelight.setCameraMode(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_limelight.periodic();
    m_limelight.setLedMode(1);
    m_limelight.setCameraMode(1);
    m_limelight.setPipeline();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_limelight.setLedMode(1);
    m_limelight.setCameraMode(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return false;

  }
}
