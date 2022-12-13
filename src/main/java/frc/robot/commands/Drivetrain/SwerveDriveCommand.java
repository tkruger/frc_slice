// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.JoystickFilter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SwerveDriveCommand extends CommandBase {
  /** Creates a new SwerveDriveCommand. */
  private final SwerveDrivetrain m_swerveDrivetrain;

  private final Joystick m_leftJoystick, m_rightJoystick;
  private final JoystickFilter translationXFilter, translationYFilter, rotationFilter;

  private final ShuffleboardTab driveTab;
  final SimpleWidget driveHeadingWidget;

  public SwerveDriveCommand(SwerveDrivetrain swerveDrivetrain, Joystick leftJoystick, Joystick rightJoystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerveDrivetrain);

    m_swerveDrivetrain = swerveDrivetrain;
    m_leftJoystick = leftJoystick;
    m_rightJoystick = rightJoystick;

    translationXFilter = new JoystickFilter(0.05, 0.3);
    translationYFilter = new JoystickFilter(0.05, 0.3);
    rotationFilter = new JoystickFilter(0.05, 0.3);

    driveTab = Shuffleboard.getTab("Driver Tab");
    driveHeadingWidget = driveTab.add("Drive Heading", 0.0).withPosition(2, 2).withSize(2, 2).withWidget("Gyro");
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    double translationX = translationXFilter.filter(m_leftJoystick.getY() * Constants.kMaxSpeedMetersPerSeconds);
    double translationY = translationYFilter.filter(m_leftJoystick.getX() * Constants.kMaxSpeedMetersPerSeconds);
    double rotation = rotationFilter.filter(m_rightJoystick.getX() * Constants.kMaxAngularVelocityRadiansPerSecond);

    m_swerveDrivetrain.swerveDrive(ChassisSpeeds.fromFieldRelativeSpeeds(
      translationX,
      translationY,
      rotation,
      new Rotation2d(Units.degreesToRadians(m_swerveDrivetrain.getHeading()))));

      driveHeadingWidget.getEntry().setDouble(m_swerveDrivetrain.getHeading());

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