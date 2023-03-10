// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class RunMandiblesCommand extends CommandBase {

  private final Intake m_intake;
  private final boolean m_runInwards;

  /** Creates a new RunMandiblesCommand. */
  public RunMandiblesCommand(Intake intake, boolean runInwards) {
    m_intake = intake;
    m_runInwards = runInwards;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(m_runInwards) {

      m_intake.runMandibles(Constants.Intake.MANDIBLE_RUN_SPEED);

    }
    else {

      m_intake.runMandibles(-Constants.Intake.MANDIBLE_RUN_SPEED);

    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    if(m_runInwards) {

      m_intake.runMandibles(Constants.Intake.MANDIBLE_RUN_SPEED);

    }
    else {

      m_intake.runMandibles(0);

    }

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return false;

  }

}
