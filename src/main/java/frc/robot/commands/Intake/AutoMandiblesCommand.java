// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.Intake;

public class AutoMandiblesCommand extends CommandBase {
  
  private final Intake m_intake;
  private final ColorSensor m_colorSensor;

  /** Creates a new AutoMandibles. */
  public AutoMandiblesCommand(Intake intake, ColorSensor colorSensor) {
    m_intake = intake;
    m_colorSensor = colorSensor;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
    // I don't think the color sensor needs a dependency here because multiple things can access the color sensor at the same time
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.setMandiblePID(Constants.Intake.MANDIBLE_KP, Constants.Intake.MANDIBLE_KI, Constants.Intake.MANDIBLE_KD);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Check if game piece is a cone
    if (m_colorSensor.getGamePiece() == 2) {
      m_intake.closeMandibles();
    }else {
      m_intake.openMandibles();
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
