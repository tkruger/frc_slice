// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;

//import edu.wpi.first.wpilibj.smartdashboard.*;

/** An example command that uses an example subsystem. */
public class AutonomousTimedDriveStraightCommand extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final Drivetrain m_drivetrain;

  private final Timer timer;
  private final double forwardSpeed, time;
  private Rotation2d startRotation;
  private final PIDController pid;


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutonomousTimedDriveStraightCommand(Drivetrain drivetrain, double forwardSpeed, double time) {
    this.m_drivetrain = drivetrain;
    this.forwardSpeed = forwardSpeed;
    this.time = time;

    timer = new Timer();

    pid = new PIDController(1, 0, 0);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_drivetrain.resetHeading();

    timer.reset();
    timer.start();

    startRotation = m_drivetrain.getRotation2d();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Sets robot speed and turn speed
    Rotation2d error = m_drivetrain.getRotation2d().minus(startRotation);
    double turnSpeed = pid.calculate(error.getRadians(), 0);


    m_drivetrain.ArcadeDrive(forwardSpeed, -turnSpeed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_drivetrain.ArcadeDrive(0, 0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return timer.get() > time;

  }
}
