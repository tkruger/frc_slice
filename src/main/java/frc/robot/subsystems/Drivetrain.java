// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  CANSparkMax leftMotorFront;
  CANSparkMax leftMotorBack;
  CANSparkMax rightMotorFront;
  CANSparkMax rightMotorBack;

  DifferentialDrive robotDrive;

  public Drivetrain() {

    final CANSparkMax leftMotorFront = new CANSparkMax(Constants.CANSPARKMAX_LEFT_MOTOR_FRONT, CANSparkMaxLowLevel.MotorType .kBrushless);
    final CANSparkMax leftMotorBack = new CANSparkMax(Constants.CANSPARKMAX_LEFT_MOTOR_BACK, CANSparkMaxLowLevel.MotorType .kBrushless);
    final CANSparkMax rightMotorFront = new CANSparkMax(Constants.CANSPARKMAX_RIGHT_MOTOR_FRONT, CANSparkMaxLowLevel.MotorType .kBrushless);
    final CANSparkMax rightMotorBack = new CANSparkMax(Constants.CANSPARKMAX_RIGHT_MOTOR_BACK, CANSparkMaxLowLevel.MotorType .kBrushless);
  
    final MotorControllerGroup leftMotors = new MotorControllerGroup(leftMotorFront, leftMotorBack);
    final MotorControllerGroup rightMotors = new MotorControllerGroup(rightMotorFront, rightMotorBack);

    final DifferentialDrive robotDrive = new DifferentialDrive(leftMotors, rightMotors);

  }


  public void arcadeDrive(double moveSpeed, double rotateSpeed) {

    robotDrive.arcadeDrive(moveSpeed, rotateSpeed);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
