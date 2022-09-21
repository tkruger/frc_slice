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
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;

import edu.wpi.first.math.StateSpaceUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.util.Units;

public class Drivetrain extends SubsystemBase {
  
  //Creates drivetrain motor objects and groups
  private final CANSparkMax leftMotorFront, leftMotorBack, rightMotorFront, rightMotorBack;
  private final MotorControllerGroup leftMotors, rightMotors;
  private final DifferentialDrive robotDrive;

  private final Gyro m_gyro;

  private final DifferentialDriveOdometry m_odometry;

  public final RelativeEncoder leftEncoderFront, leftEncoderBack, rightEncoderFront, rightEncoderBack;

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

    leftMotorFront.setControlFramePeriodMs(250);
    leftMotorBack.setControlFramePeriodMs(250);
    rightMotorFront.setControlFramePeriodMs(250);
    rightMotorBack.setControlFramePeriodMs(250);

    leftMotorFront.setIdleMode(CANSparkMax.IdleMode.kBrake);
    leftMotorBack.setIdleMode(CANSparkMax.IdleMode.kBrake);
    rightMotorFront.setIdleMode(CANSparkMax.IdleMode.kBrake);
    rightMotorBack.setIdleMode(CANSparkMax.IdleMode.kBrake);

    robotDrive = new DifferentialDrive(leftMotors, rightMotors);
    
    rightMotors.setInverted(true);

    //(NOTE TO SELF) Check what type of gyroscope we use.
    m_gyro = new ADXRS450_Gyro();

    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());

    //(NOTE TO SELF) Check if we use quadature encoders
    leftEncoderFront = leftMotorFront.getEncoder();
    leftEncoderBack = leftMotorBack.getEncoder();

    rightEncoderFront = rightMotorFront.getEncoder();
    rightEncoderBack = rightMotorBack.getEncoder();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  //(NOTE TO SELF) The standard deviations values are place holders and should be measured proplerly for our robot
  public Pose2d getEstimatedGlobalPose(Pose2d estimatedRobotPose) {

    var rand = 
      StateSpaceUtil.makeWhiteNoiseVector(VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(5)));

    return new Pose2d(
        estimatedRobotPose.getX() + rand.get(0, 0),
        estimatedRobotPose.getY() + rand.get(1, 0),
        estimatedRobotPose.getRotation().plus(new Rotation2d(rand.get(2, 0))));

  }

  public Pose2d updateOdometry() {

    double leftEncodersDistance = leftEncoderFront.getPosition() + leftEncoderFront.getPosition();
    double rightEncodersDistance = rightEncoderFront.getPosition() + rightEncoderBack.getPosition();

    return
      m_odometry.update(
        m_gyro.getRotation2d(), leftEncodersDistance, rightEncodersDistance);

  }

  public double[] updateVelocities() {

    return new double[] {
      leftEncoderFront.getVelocity(), 
      leftEncoderBack.getVelocity(), 
      rightEncoderFront.getVelocity(), 
      rightEncoderBack.getVelocity()};

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

  public void resetOdometry() {

    m_odometry.resetPosition(m_odometry.getPoseMeters(), m_gyro.getRotation2d());

  }

  public void resetEncoders() {

    leftEncoderFront.setPosition(0);
    leftEncoderBack.setPosition(0);
    rightEncoderFront.setPosition(0);
    rightEncoderBack.setPosition(0);

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