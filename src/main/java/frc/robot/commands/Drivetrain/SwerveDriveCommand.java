// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import frc.robot.Constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrivetrain;

public class SwerveDriveCommand extends CommandBase {
  /** Creates a new SwerveDriveCommand. */
  SwerveDrivetrain m_swerveDrivetrain;
  Joystick m_leftJoystick, m_rightJoystick;

  public SwerveDriveCommand(SwerveDrivetrain swerveDrivetrain, Joystick leftJoystick, Joystick rightJoystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveDrivetrain);

    m_swerveDrivetrain = swerveDrivetrain;
    m_leftJoystick = leftJoystick;
    m_rightJoystick = rightJoystick;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    double translationX = -m_swerveDrivetrain.modifyAxis(m_leftJoystick.getY() * Constants.kMaxSpeedMetersPerSeconds);
    double translationY = m_swerveDrivetrain.modifyAxis(m_leftJoystick.getX() * Constants.kMaxSpeedMetersPerSeconds);
    double rotation = m_swerveDrivetrain.modifyAxis(m_rightJoystick.getX() * Constants.kMaxAngularVelocityRadiansPerSecond);

    m_swerveDrivetrain.swerveDrive(ChassisSpeeds.fromFieldRelativeSpeeds(
      translationX,
      translationY,
      rotation,
      new Rotation2d(Units.degreesToRadians(m_swerveDrivetrain.getHeading()))));

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