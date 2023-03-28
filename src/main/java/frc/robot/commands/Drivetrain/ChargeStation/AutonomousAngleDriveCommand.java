// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain.ChargeStation;

import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj2.command.CommandBase;

//import edu.wpi.first.wpilibj.smartdashboard.*;

/** An example command that uses an example subsystem. */
public class AutonomousAngleDriveCommand extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final Drivetrain m_drivetrain;

  private final double forwardSpeed;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutonomousAngleDriveCommand(Drivetrain drivetrain, double forwardSpeed) {
    this.m_drivetrain = drivetrain;
    this.forwardSpeed = forwardSpeed;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_drivetrain.resetHeading();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Sets robot speed and turn speed

    m_drivetrain.ArcadeDrive(forwardSpeed, 0.1);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_drivetrain.ArcadeDrive(0, 0);
    System.out.println("End of Autonomous Angle Drive Command");

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    double angle = m_drivetrain.getRoll();
    return Math.abs(angle) > 14;

  }
}
