// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

/**
 * A simple command that tells the mandibles on the intake to close, then immediately ends
 */
public class CloseMandiblesCommand extends CommandBase {

  private final Intake m_intake;

  /** Creates a new OpenMandibles.
   * @param intake the intake subsystem it controls
   */
  public CloseMandiblesCommand(Intake intake) {
    m_intake = intake;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.setMandiblePID(Constants.Intake.MANDIBLE_KP, Constants.Intake.MANDIBLE_KI, Constants.Intake.MANDIBLE_KD);
    m_intake.closeMandibles();
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
