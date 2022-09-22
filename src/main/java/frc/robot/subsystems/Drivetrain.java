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
    m_odometry.update(
      m_gyro.getRotation2d(),
      getAverageLeftEncoderDistance(),
      getAverageRightEncoderDistance());

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

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
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

  public void resetOdometry() {

    m_odometry.resetPosition(m_odometry.getPoseMeters(), m_gyro.getRotation2d());

  }

  public void resetEncoders() {

    leftEncoderFront.setPosition(0);
    leftEncoderBack.setPosition(0);
    rightEncoderFront.setPosition(0);
    rightEncoderBack.setPosition(0);

  }

    /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (getAverageLeftEncoderDistance() + getAverageRightEncoderDistance()) / 2.0;
  }

  public double getAverageLeftEncoderDistance() {
      return (leftEncoderFront.getPosition() + leftEncoderBack.getPosition()) / 2.0; 
  }

  public double getAverageRightEncoderDistance() {
    return (rightEncoderFront.getPosition() + rightEncoderBack.getPosition()) / 2.0; 
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