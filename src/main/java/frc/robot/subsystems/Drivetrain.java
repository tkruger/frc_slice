// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;

import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
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

  Gyro m_gyro;
  DifferentialDriveOdometry m_odometry;

  Encoder frontLeftEncoder;
  Encoder backLeftEncoder;
  Encoder frontRightEncoder;
  Encoder backRightEncoder;

  public Drivetrain() {

    final CANSparkMax leftMotorFront = new CANSparkMax(Constants.CANSPARKMAX_LEFT_MOTOR_FRONT, CANSparkMaxLowLevel.MotorType .kBrushless);
    final CANSparkMax leftMotorBack = new CANSparkMax(Constants.CANSPARKMAX_LEFT_MOTOR_BACK, CANSparkMaxLowLevel.MotorType .kBrushless);
    final CANSparkMax rightMotorFront = new CANSparkMax(Constants.CANSPARKMAX_RIGHT_MOTOR_FRONT, CANSparkMaxLowLevel.MotorType .kBrushless);
    final CANSparkMax rightMotorBack = new CANSparkMax(Constants.CANSPARKMAX_RIGHT_MOTOR_BACK, CANSparkMaxLowLevel.MotorType .kBrushless);
  
    final MotorControllerGroup leftMotors = new MotorControllerGroup(leftMotorFront, leftMotorBack);
    final MotorControllerGroup rightMotors = new MotorControllerGroup(rightMotorFront, rightMotorBack);

    final DifferentialDrive robotDrive = new DifferentialDrive(leftMotors, rightMotors);

    rightMotors.setInverted(true);

    final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
    final Gyro m_gyro = new ADXRS450_Gyro();

    final Encoder frontLeftEncoder = 
    new Encoder(
      Constants.DRIVETRAIN_FRONT_LEFT_ENCODER[0], 
      Constants.DRIVETRAIN_FRONT_LEFT_ENCODER[1], 
      Constants.DRIVETRAIN_FRONT_LEFT_ENCODER_REVERSED);
    final Encoder backLeftEncoder = 
    new Encoder(
      Constants.DRIVETRAIN_BACK_LEFT_ENCODER[0], 
      Constants.DRIVETRAIN_BACK_LEFT_ENCODER[1], 
      Constants.DRIVETRAIN_BACK_LEFT_ENCODER_REVERSED);
    final Encoder frontRightEncoder = 
    new Encoder(
      Constants.DRIVETRAIN_FRONT_RIGHT_ENCODER[0], 
      Constants.DRIVETRAIN_FRONT_RIGHT_ENCODER[1], 
      Constants.DRIVETRAIN_FRONT_RIGHT_ENCODER_REVERSED);
    final Encoder backRightEncoder = 
    new Encoder(
      Constants.DRIVETRAIN_BACK_RIGHT_ENCODER[0], 
      Constants.DRIVETRAIN_BACK_RIGHT_ENCODER[1], 
      Constants.DRIVETRAIN_BACK_RIGHT_ENCODER_REVERSED);

    frontLeftEncoder.setDistancePerPulse(Constants.DRIVETRAIN_ENCODER_DISTANCE_PER_PULSE);
    backLeftEncoder.setDistancePerPulse(Constants.DRIVETRAIN_ENCODER_DISTANCE_PER_PULSE);
    frontRightEncoder.setDistancePerPulse(Constants.DRIVETRAIN_ENCODER_DISTANCE_PER_PULSE);
    backRightEncoder.setDistancePerPulse(Constants.DRIVETRAIN_ENCODER_DISTANCE_PER_PULSE);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_odometry.update(
      m_gyro.getRotation2d(),
        frontLeftEncoder.getRate() +
        backLeftEncoder.getRate() / 2,
        frontRightEncoder.getRate() +
        backRightEncoder.getRate() / 2);

  }

  public Pose2d getPose() {

    return m_odometry.getPoseMeters();

  }

  public void resetOdometry(Pose2d pose) {

    pose = m_odometry.getPoseMeters();
    
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());

  }

  /**
   * Drives the robot at given x, y and theta speeds. Speeds range from [-1, 1] and the linear
   * speeds have no effect on the angular speed.
   *
   * @param moveSpeed Speed of the robot moving in the x and y directions direction (left/right/forwards/bacwards).
   * @param rotateSpeed Speed of the robot rotating.
   * @param fieldRelative Whether the provided move and rotate speeds are relative to the field.
   */

  public void arcadeDrive(double moveSpeed, double rotateSpeed) {

    robotDrive.arcadeDrive(moveSpeed, rotateSpeed);

  }


}
