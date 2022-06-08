// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
//import frc.robot.Constants;
import frc.robot.RobotContainer;

public class DriveArcade extends CommandBase {
  /** Creates a new DriveArcade. */
  public DriveArcade() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  
    //(NOTE TO SELF) Check if getRawAxis() or getX()/getY() should be used.
    //(NOTE TO SELF) Remove constants for getRawAxis if decided to not be used.
    double moveSpeed = -RobotContainer.leftJoystick.getX();
    double rotateSpeed = RobotContainer.rightJoystick.getY();

    RobotContainer.m_drivetrain.arcadeDrive(moveSpeed, rotateSpeed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    RobotContainer.m_drivetrain.arcadeDrive(0, 0);
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return false;

  }
}
