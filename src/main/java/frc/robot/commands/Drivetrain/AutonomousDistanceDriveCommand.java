// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import frc.robot.Constants;
import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj2.command.CommandBase;

//import edu.wpi.first.wpilibj.smartdashboard.*;

/** An example command that uses an example subsystem. */
public class AutonomousDistanceDriveCommand extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final Drivetrain m_drivetrain;

  private final double forwardSpeed, distance;

  private double targetDistance;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutonomousDistanceDriveCommand(Drivetrain drivetrain, double forwardSpeed, double distance) {
    this.m_drivetrain = drivetrain;
    this.forwardSpeed = forwardSpeed;
    this.distance = -1 * Math.abs(distance) * Math.signum(forwardSpeed);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_drivetrain.resetHeading();
    targetDistance = m_drivetrain.getAverageDistance() + distance;
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Sets robot speed and turn speed

    m_drivetrain.ArcadeDrive(forwardSpeed, 0);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_drivetrain.ArcadeDrive(0, 0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    double error = Math.abs(m_drivetrain.getAverageDistance() - targetDistance); 
    return error < Constants.Drivetrain.AUTO_DISTANCE_ERROR_TOLERANCE;

  }
}