// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import frc.robot.RobotContainer;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class DrivetrainCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Drivetrain m_drivetrain;

  private final Joystick leftJoystick, rightJoystick;

  /**
   * Creates a new ExampleCommand.
   *
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
    RobotContainer.m_drivetrain.ArcadeDrive(forwardSpeed, turnSpeed);

    //Updates odometry
    RobotContainer.m_drivetrain.periodic();

    //Prints out odometry information
    System.out.println(m_drivetrain.getPose());
    System.out.println(m_drivetrain.getHeading());
    System.out.println(m_drivetrain.getTurnRate());

    //Prints out wheel speeds
    System.out.println(m_drivetrain.getCurrentWheelSpeeds());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_drivetrain.ArcadeDrive(0, 0);

    //Resets gyro
    m_drivetrain.zeroHeading();

    //Resets encoder count 
    m_drivetrain.resetEncoders();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return false;
    
  }
}
