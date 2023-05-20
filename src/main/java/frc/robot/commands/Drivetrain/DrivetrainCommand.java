// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import frc.robot.JoystickFilter;
import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Joystick;

//import edu.wpi.first.wpilibj.smartdashboard.*;

/** An example command that uses an example subsystem. */
public class DrivetrainCommand extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final Drivetrain m_drivetrain;

  private final Joystick leftJoystick;
  private final Joystick rightJoystick;

  private final JoystickFilter forwardFilter, turnFilter;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DrivetrainCommand(Drivetrain drivetrain, Joystick leftJoystick, Joystick rightJoystick) {
    this.m_drivetrain = drivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);

    this.leftJoystick = leftJoystick;
    this.rightJoystick = rightJoystick;

    forwardFilter = new JoystickFilter(0.05, 0.3, false);
    turnFilter = new JoystickFilter(0.05, 0.3);

    m_drivetrain.resetHeading();

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_drivetrain.resetOdometry(Limelight.getLastBotPoseBlue());

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Sets robot speed and turn speed
    double forwardSpeed = forwardFilter.filter(leftJoystick.getY());
    double turnSpeed = turnFilter.withDead(rightJoystick.getX());

    m_drivetrain.ArcadeDrive(forwardSpeed, turnSpeed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_drivetrain.ArcadeDrive(0, 0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return false;

  }
}
