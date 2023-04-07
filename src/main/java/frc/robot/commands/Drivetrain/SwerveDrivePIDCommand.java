// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.JoystickFilter;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveDrivePIDCommand extends CommandBase {
  /** Creates a new SwerveDriveCommand. */
  private final Drivetrain m_drivetrain;

  private final Joystick m_leftJoystick, m_rightJoystick;
  private final JoystickFilter translationXFilter, translationYFilter, rotationFilter;

  public SwerveDrivePIDCommand(Drivetrain drivetrain, Joystick leftJoystick, Joystick rightJoystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);

    m_drivetrain = drivetrain;
    m_leftJoystick = leftJoystick;
    m_rightJoystick = rightJoystick;

    translationXFilter = new JoystickFilter(0.07, 0.3);
    translationYFilter = new JoystickFilter(0.07, 0.3);
    rotationFilter = new JoystickFilter(0.07, 0.3);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_drivetrain.resetHeading();

    m_drivetrain.setSteerPID(1.0, 0, 0.1);
    //These PIDF gains are placeholders for now
    m_drivetrain.setDrivePIDF(0, 0, 0, 0);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
       
    double translationX = translationXFilter.filter(m_leftJoystick.getY()) * Constants.kDrivetrain.kMaxVelocityMetersPerSecond;
    double translationY = translationYFilter.filter(m_leftJoystick.getX()) * Constants.kDrivetrain.kMaxVelocityMetersPerSecond;
    double rotation = rotationFilter.filter(m_rightJoystick.getX()) * Constants.kDrivetrain.kMaxAngularVelocityRadiansPerSecond;

    m_drivetrain.swerveDrivePID(
      translationX,
      translationY,
      rotation);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_drivetrain.swerveDrivePID(0, 0, 0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return false;

  }

}