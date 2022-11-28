// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Motors;

public class MotorCommand extends CommandBase {
  private final Motors m_Motors;

  /** Creates a new MotorCommand. */
  public MotorCommand(Motors motors) {
    m_Motors = motors;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Motors);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Motors.runMotors(Constants.yawMotor_percentOut, Constants.pitchMotor_outPutVolts);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Motors.runMotors(0, 0); //Stops the motors
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
