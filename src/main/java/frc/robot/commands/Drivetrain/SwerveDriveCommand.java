// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.JoystickFilter;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveDriveCommand extends CommandBase {
  /** Creates a new SwerveDriveCommand. */
  private final Drivetrain m_drivetrain;

  private final Joystick m_leftJoystick, m_rightJoystick;
  private final JoystickFilter translationXFilter, translationYFilter, rotationFilter;

  private final boolean m_isOpenLoop;

  public SwerveDriveCommand(Drivetrain drivetrain, Joystick leftJoystick, Joystick rightJoystick, boolean isOpenLoop) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);

    m_drivetrain = drivetrain;

    m_leftJoystick = leftJoystick;
    m_rightJoystick = rightJoystick;

    m_isOpenLoop = isOpenLoop;

    translationXFilter = new JoystickFilter(0.07, 0.3);
    translationYFilter = new JoystickFilter(0.07, 0.3);
    rotationFilter = new JoystickFilter(0.07, 0.3);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_drivetrain.resetHeading();

    m_drivetrain.setAnglePIDF(0.01, 0, 0, 0);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    double translationX = translationXFilter.filter(m_leftJoystick.getY()) * Constants.kDrivetrain.MAX_VELOCITY;
    double translationY = translationYFilter.filter(m_leftJoystick.getX()) * Constants.kDrivetrain.MAX_VELOCITY;
    double rotation = rotationFilter.filter(m_rightJoystick.getX()) * Constants.kDrivetrain.MAX_ANGULAR_VELOCITY;

    m_drivetrain.swerveDrive(
      translationX,
      translationY,
      rotation,
      m_isOpenLoop);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_drivetrain.swerveDrive(0, 0, 0, m_isOpenLoop);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return false;

  }

}