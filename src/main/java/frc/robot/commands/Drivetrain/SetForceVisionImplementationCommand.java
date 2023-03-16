// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class SetForceVisionImplementationCommand extends CommandBase {

  private final Drivetrain m_drivetrain;
  private final boolean m_enable;

  /** Creates a new ForceVisionImplementation. */
  public SetForceVisionImplementationCommand(Drivetrain drivetrain, boolean enable) {

    m_drivetrain = drivetrain;
    m_enable = enable;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    if(m_enable) {

      m_drivetrain.enableForceVisionImplementation();

    }
    else {

      m_drivetrain.disableForceVisionImplementation();

    }

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
