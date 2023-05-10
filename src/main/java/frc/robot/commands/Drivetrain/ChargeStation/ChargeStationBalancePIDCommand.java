// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain.ChargeStation;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class ChargeStationBalancePIDCommand extends CommandBase {

  private final Drivetrain m_drivetrain;

  private double pitch;

  private final PIDController pidController;

  /** Creates a new ChargeStationBalanceCommand. */
  public ChargeStationBalancePIDCommand(Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);

    m_drivetrain = drivetrain;

    //pidController = new PIDController(0.032, 0.0001, 0.0009);
    pidController = new PIDController(0.032, 0.0001, 0.0009);
    pidController.setSetpoint(0);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_drivetrain.setDriveIdleMode(true);
    m_drivetrain.setAngleIdleMode(true);

    m_drivetrain.setDrivePIDF(.17, .000001, .12, .62);
    m_drivetrain.setDrivePIDF(.17, .000001, .12, .62);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    pitch = m_drivetrain.getRoll();

    //if (Math.abs(pitch) > 2) {
    if (Math.abs(pitch) > 0.5) {  
      m_drivetrain.swerveDrive(pidController.calculate(pitch), 0, 0, true);
    } else {
      m_drivetrain.swerveDrive(0, 0, 0, true);
      pidController.reset();
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