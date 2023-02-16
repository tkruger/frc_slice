// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

/**
 * A simple command that tells the mandibles on the intake to open, then immediately ends
 */
public class OpenMandibles extends CommandBase {

  private final Intake m_intake;

  /** Creates a new OpenMandibles.
   * @param intake the intake subsystem it controls
   */
  public OpenMandibles(Intake intake) {
    m_intake = intake;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.setMandiblePID(Constants.intake_MANDIBLE_KP, Constants.intake_MANDIBLE_KI, Constants.intake_MANDIBLE_KD);
    m_intake.openMandibles();
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
    return true;
  }
}
