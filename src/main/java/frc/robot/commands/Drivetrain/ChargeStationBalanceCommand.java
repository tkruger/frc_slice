// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class ChargeStationBalanceCommand extends CommandBase {
  /** Creates a new ChargeStationBalanceCommand. */
private final Drivetrain m_drivetrain;

private boolean docked;
private double pitch;

  public ChargeStationBalanceCommand(Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = drivetrain;

    addRequirements(drivetrain);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    docked = false;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    pitch = m_drivetrain.getPitch();

    if(pitch > 2) {

      docked = true;

      m_drivetrain.curvatureDrive(0.3, 0);

    }

    if(docked == false) {

      m_drivetrain.curvatureDrive(0.3, 0);

    }

    if(pitch < -2) {

      m_drivetrain.curvatureDrive(-0.3, 0);

    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_drivetrain.curvatureDrive(0, 0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return (docked == true && Math.abs(pitch) < 2);
    
  }
}
