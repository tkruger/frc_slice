// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.wpilibj.Joystick;

//import edu.wpi.first.wpilibj.smartdashboard.*;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** An example command that uses an example subsystem. */
public class DrivetrainCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Drivetrain m_drivetrain;

  private final Joystick leftJoystick;
  private final Joystick rightJoystick;

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

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    //Resets gyro heading, encoder positions, and pose reading
    m_drivetrain.resetOdometry();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    //Sets robot speed and turn speed
    double forwardSpeed = leftJoystick.getY();
    double turnSpeed = rightJoystick.getX();

    m_drivetrain.ArcadeDrive(forwardSpeed, turnSpeed);

    //Updates the odometry with a new estimated robot pose
    m_drivetrain.updateOdometry();

    //Prints out the estimated robot pose
    System.out.println(m_drivetrain.updateOdometry());

    //Prints out the rotation 2d heading
    SmartDashboard.putNumber("Drivetrain Heading:", m_drivetrain.getHeading());

    //Prints out gyro turn rate
    SmartDashboard.putNumber("Drivetrain Turn Rate:", m_drivetrain.getTurnRate());

    //Prints out left front motor velocity
    SmartDashboard.putNumber("Left Front Motor Velocity:", m_drivetrain.getLeftFrontVelocity());

    //Prints out left back motor velocity
    SmartDashboard.putNumber("Left Back Motor Velocity:", m_drivetrain.getLeftBackVelocity());

    //Prints out right front motor velocity
    SmartDashboard.putNumber("Right Front Motor Velocity:", m_drivetrain.getRightFrontVelocity());

    //Prints out right back motor velocity
    SmartDashboard.putNumber("Right Back Motor Velocity:", m_drivetrain.getRightBackVelocity());

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
