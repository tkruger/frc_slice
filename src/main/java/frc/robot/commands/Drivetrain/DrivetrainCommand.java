// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** A Drivetrain command that uses a drivetrain subsystem. */
public class DrivetrainCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Drivetrain m_drivetrain;

  private final Joystick leftJoystick, rightJoystick;

  /**
   * @param subsystem The subsystem used by this command.
   */
  public DrivetrainCommand(Drivetrain drivetrain, Joystick leftJoystick, Joystick rightJoystick) {
    m_drivetrain = drivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);

    this.leftJoystick = leftJoystick;
    this.rightJoystick = rightJoystick;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Sets robot speed and turn speed
    double forwardSpeed = leftJoystick.getY();
    double turnSpeed = rightJoystick.getX();

    if(!leftJoystick.getRawButton(1) && !rightJoystick.getRawButton(1)) {

      m_drivetrain.ArcadeDrive(forwardSpeed, turnSpeed);

    }

    //Updates and prints out encoder speeds and rotation 2d heading
    System.out.println(m_drivetrain.getEstimatedGlobalPose(m_drivetrain.updateOdometry()));

    //Prints out rotation 2d heading in degrees
    SmartDashboard.putNumber("Drivetrain Heading", m_drivetrain.getHeading());

    //Prints out gyro turn rate
    SmartDashboard.putNumber("Drivetrain Turn Rate", m_drivetrain.getTurnRate());

    //Prints out motor velocities
    SmartDashboard.putNumberArray("Drivetrain Motor Velocities", m_drivetrain.updateVelocities());
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_drivetrain.ArcadeDrive(0, 0);

    //Resets gyro
    m_drivetrain.zeroHeading();

    //Resets encoder counts
    m_drivetrain.resetEncoders();

    //Resets odometry position
    m_drivetrain.resetOdometry();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return false;
    
  }
}
