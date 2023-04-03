// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.JoystickFilter;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveDrivePIDCommand extends CommandBase {
  /** Creates a new SwerveDriveCommand. */
  private final SwerveDrivetrain m_swerveDrivetrain;

  private final Joystick m_leftJoystick, m_rightJoystick;
  private final JoystickFilter translationXFilter, translationYFilter, rotationFilter;

  public SwerveDrivePIDCommand(SwerveDrivetrain swerveDrivetrain, Joystick leftJoystick, Joystick rightJoystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveDrivetrain);

    m_swerveDrivetrain = swerveDrivetrain;
    m_leftJoystick = leftJoystick;
    m_rightJoystick = rightJoystick;

    translationXFilter = new JoystickFilter(0.07, 0.3);
    translationYFilter = new JoystickFilter(0.07, 0.3);
    rotationFilter = new JoystickFilter(0.07, 0.3);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_swerveDrivetrain.resetHeading();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
       
    double translationX = translationXFilter.filter(m_leftJoystick.getY()) * Constants.Drivetrain.kMaxVelocityMetersPerSecond;
    double translationY = translationYFilter.filter(m_leftJoystick.getX()) * Constants.Drivetrain.kMaxVelocityMetersPerSecond;
    double rotation = rotationFilter.filter(m_rightJoystick.getX()) * Constants.Drivetrain.kMaxAngularVelocityRadiansPerSecond;

    m_swerveDrivetrain.swerveDrivePID(
      translationX,
      translationY,
      rotation);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_swerveDrivetrain.swerveDrivePID(0, 0, 0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return false;

  }

}