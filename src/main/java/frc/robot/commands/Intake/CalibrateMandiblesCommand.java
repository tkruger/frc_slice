// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class CalibrateMandiblesCommand extends CommandBase {
  
  private final Intake m_intake;
  /** Creates a new CalibrateMandiblesCommand. */
  public CalibrateMandiblesCommand(Intake intake) {
    m_intake = intake;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.callibratePreparation();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.runMandibles(Constants.Intake.CALIBRATION_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.runMandibles(0);
    m_intake.setMandibleEncoderPosition(Constants.Intake.MANDIBLE_CLOSED_POSITION);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_intake.mandibleVoltageSpike();
  }
}