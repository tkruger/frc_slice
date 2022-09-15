// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.revrobotics.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;


public class Drivetrain extends SubsystemBase {
  
  //Creates drivetrain motor objects and groups
  private final CANSparkMax leftMotorFront, leftMotorBack, rightMotorFront, rightMotorBack;
  private final MotorControllerGroup leftMotors, rightMotors;
  private final DifferentialDrive robotDrive;

  private final Gyro m_gyro;

  private final DifferentialDriveOdometry m_odometry;

  private final Encoder leftEncoder;
  private final Encoder rightEncoder;


  /** Creates a new Drivetrain. */
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

    //(NOTE TO SELF) Check what type of gyroscope we use.
    m_gyro = new ADXRS450_Gyro();

    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());

    //(NOTE TO SELF) Check what ports are used by the encoders if the robot has them.
    leftEncoder = 
    new Encoder(
      Constants.drivetrain_LEFT_ENCODER[0], 
      Constants.drivetrain_LEFT_ENCODER[1], 
      Constants.drivetrain_LEFT_ENCODER_REVERSED);
    rightEncoder = 
    new Encoder(
      Constants.drivetrain_RIGHT_ENCODER[0], 
      Constants.drivetrain_RIGHT_ENCODER[1], 
      Constants.drivetrain_RIGHT_ENCODER_REVERSED);

    leftEncoder.setDistancePerPulse(Constants.drivetrain_ENCODER_DISTANCE_PER_PULSE);
    rightEncoder.setDistancePerPulse(Constants.drivetrain_ENCODER_DISTANCE_PER_PULSE);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_odometry.update(
      m_gyro.getRotation2d(),
      leftEncoder.getDistance(),
      rightEncoder.getDistance());

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
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
   * @param forwardSpeed Speed of the robot moving in the x and y directions direction (left/right/forwards/bacwards).
   * @param rotateSpeed Speed of the robot rotating.
   * @param fieldRelative Whether the provided move and rotate speeds are relative to the field.
   */

  public void ArcadeDrive(double forwardSpeed, double turnSpeed) { 

    robotDrive.arcadeDrive(forwardSpeed, turnSpeed);

  }

    /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMotors.setVoltage(leftVolts);
    rightMotors.setVoltage(rightVolts);
    robotDrive.feed();
  }

  public void resetEncoders() {

    leftEncoder.reset();
    rightEncoder.reset();

  }

  public DifferentialDriveWheelSpeeds getCurrentWheelSpeeds() {

    return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());

  }

    /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (leftEncoder.getDistance() + rightEncoder.getDistance()) / 2.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public Encoder getLeftEncoder() {
    return leftEncoder;
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public Encoder getRightEncoder() {
    return rightEncoder;
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    robotDrive.setMaxOutput(maxOutput);
  }

  public void zeroHeading() {

    m_gyro.reset();

  }

  public double getHeading() {

    return m_gyro.getRotation2d().getDegrees();

  }

  public double getTurnRate() {

    return -m_gyro.getRate();

  }

}