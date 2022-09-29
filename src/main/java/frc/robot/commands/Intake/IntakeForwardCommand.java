// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;
//import com.ctre.phoenix.motorcontrol.can.TalonFX;
//import frc.robot.subsystems.Intake;

public class IntakeForwardCommand extends CommandBase {

  Intake m_intake;

  boolean intakeForwardPressed = false;
  boolean intakeForwardToggle = false;
  boolean intakeBackward = false;

  public IntakeForwardCommand(Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);

    m_intake = intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    intakeForwardToggle = true;

    m_intake.runIntake(intakeForwardToggle, intakeBackward);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeForwardToggle = false;
    m_intake.runIntake(intakeForwardToggle, intakeBackward);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
