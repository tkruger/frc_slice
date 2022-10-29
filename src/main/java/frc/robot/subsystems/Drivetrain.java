// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

import com.revrobotics.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  private final CANSparkMax leftMotorFront, leftMotorBack, rightMotorFront, rightMotorBack;
  private final MotorControllerGroup leftMotors, rightMotors;
  private final DifferentialDrive robotDrive;

  public Drivetrain() {
    //Instantiates motors and motor groups
    leftMotorFront = new CANSparkMax(Constants.drivetrain_LEFT_FRONT_PORT, MotorType.kBrushless);
    leftMotorBack = new CANSparkMax(Constants.drivetrain_LEFT_BACK_PORT, MotorType.kBrushless);
    rightMotorFront = new CANSparkMax(Constants.drivetrain_RIGHT_FRONT_PORT, MotorType.kBrushless);
    rightMotorBack = new CANSparkMax(Constants.drivetrain_RIGHT_BACK_PORT, MotorType.kBrushless);

    leftMotors = new MotorControllerGroup(leftMotorFront, leftMotorBack);
    rightMotors = new MotorControllerGroup(rightMotorFront, rightMotorBack);

    leftMotorFront.restoreFactoryDefaults();
    leftMotorBack.restoreFactoryDefaults();
    rightMotorFront.restoreFactoryDefaults();
    rightMotorBack.restoreFactoryDefaults();

    leftMotorFront.setIdleMode(CANSparkMax.IdleMode.kBrake);
    leftMotorBack.setIdleMode(CANSparkMax.IdleMode.kBrake);
    rightMotorFront.setIdleMode(CANSparkMax.IdleMode.kBrake);
    rightMotorBack.setIdleMode(CANSparkMax.IdleMode.kBrake);

    robotDrive = new DifferentialDrive(leftMotors, rightMotors);
    
    rightMotors.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void ArcadeDrive(double forwardSpeed, double turnSpeed) { 

    robotDrive.arcadeDrive(-forwardSpeed, turnSpeed);

  }
}
