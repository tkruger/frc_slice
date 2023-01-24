/// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.networktables.GenericEntry;

import com.revrobotics.*;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;

public class Drivetrain extends SubsystemBase {

  // Creates drivetrain motor objects and groups
  private final CANSparkMax leftMotorFront, leftMotorBack, rightMotorFront, rightMotorBack;
  private final MotorControllerGroup leftMotors, rightMotors;
  private final DifferentialDrive robotDrive;

  private final AHRS navXGyro;

  public final RelativeEncoder leftEncoderFront, leftEncoderBack, rightEncoderFront, rightEncoderBack;

  // PID Controllers
  public final SparkMaxPIDController leftPIDFront, leftPIDBack, rightPIDFront, rightPIDBack;

  private final DifferentialDriveOdometry m_odometry;

  private final ShuffleboardTab driveTab;

  //private final GenericEntry driveHeadingWidget;

  private final Field2d m_field2d;

  /** Creates a new Drivetrain. */
  public Drivetrain() {

    // Instantiates motors and motor groups
    leftMotorFront = new CANSparkMax(Constants.drivetrain_LEFT_FRONT_PORT, MotorType.kBrushless);
    leftMotorBack = new CANSparkMax(Constants.drivetrain_LEFT_BACK_PORT, MotorType.kBrushless);
    rightMotorFront = new CANSparkMax(Constants.drivetrain_RIGHT_FRONT_PORT, MotorType.kBrushless);
    rightMotorBack = new CANSparkMax(Constants.drivetrain_RIGHT_BACK_PORT, MotorType.kBrushless);

    leftMotors = new MotorControllerGroup(leftMotorFront, leftMotorBack);
    rightMotors = new MotorControllerGroup(rightMotorFront, rightMotorBack);

    rightMotors.setInverted(true);

    leftMotorFront.restoreFactoryDefaults();
    leftMotorBack.restoreFactoryDefaults();
    rightMotorFront.restoreFactoryDefaults();
    rightMotorBack.restoreFactoryDefaults();

    leftMotorFront.setIdleMode(CANSparkMax.IdleMode.kBrake);
    leftMotorBack.setIdleMode(CANSparkMax.IdleMode.kBrake);
    rightMotorFront.setIdleMode(CANSparkMax.IdleMode.kBrake);
    rightMotorBack.setIdleMode(CANSparkMax.IdleMode.kBrake);

    robotDrive = new DifferentialDrive(leftMotors, rightMotors);

    leftEncoderFront = leftMotorFront.getEncoder(
      SparkMaxRelativeEncoder.Type.kHallSensor,
      Constants.drivetrain_ENCODER_CPR);
    leftEncoderBack = leftMotorBack.getEncoder(
      SparkMaxRelativeEncoder.Type.kHallSensor,
      Constants.drivetrain_ENCODER_CPR);
    rightEncoderFront = rightMotorFront.getEncoder(
      SparkMaxRelativeEncoder.Type.kHallSensor,
      Constants.drivetrain_ENCODER_CPR);
    rightEncoderBack = rightMotorBack.getEncoder(
      SparkMaxRelativeEncoder.Type.kHallSensor,
      Constants.drivetrain_ENCODER_CPR);

    leftEncoderFront.setVelocityConversionFactor(Constants.drivetrain_VELOCITY_CONVERSION_FACTOR);
    leftEncoderBack.setVelocityConversionFactor(Constants.drivetrain_VELOCITY_CONVERSION_FACTOR);
    rightEncoderFront.setVelocityConversionFactor(Constants.drivetrain_VELOCITY_CONVERSION_FACTOR);
    rightEncoderBack.setVelocityConversionFactor(Constants.drivetrain_VELOCITY_CONVERSION_FACTOR);

    leftEncoderFront.setPositionConversionFactor(Constants.drivetrain_POSITION_CONVERSION_FACTOR);
    leftEncoderBack.setPositionConversionFactor(Constants.drivetrain_POSITION_CONVERSION_FACTOR);
    rightEncoderFront.setPositionConversionFactor(Constants.drivetrain_POSITION_CONVERSION_FACTOR);
    rightEncoderBack.setPositionConversionFactor(Constants.drivetrain_POSITION_CONVERSION_FACTOR);

    leftPIDFront = leftMotorFront.getPIDController();
    leftPIDBack = leftMotorBack.getPIDController();
    rightPIDFront = rightMotorFront.getPIDController();
    rightPIDBack = rightMotorBack.getPIDController();
    
    navXGyro = new AHRS(SerialPort.Port.kUSB1);

    m_field2d = new Field2d();

    resetEncoders();

    m_odometry = new DifferentialDriveOdometry(
      Rotation2d.fromDegrees(getHeading()), 
      getLeftSideDistance(), 
      getRightSideDistance());


    //Creates the "Driver Tab" on Shuffleboard
    driveTab = Shuffleboard.getTab("Driver Tab");

    //Creates a gyro widget for showing the gyro heading
    //driveHeadingWidget = driveTab.add("Drive Heading", 0.0).withWidget(BuiltInWidgets.kGyro).getEntry();

    //Displays how the robot is moving on Shuffleboard
    driveTab.add(robotDrive);

    //Sends the Fiel2d object to NetworkTables
    SmartDashboard.putData(m_field2d);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    updateOdometry();

    m_field2d.setRobotPose(getPose());

    SmartDashboard.putNumber("Left Side Position", getLeftSideDistance());
    SmartDashboard.putNumber("Right Side Position", getRightSideDistance());

    SmartDashboard.putNumber("Left Side Velocity", getLeftSideVelocity());
    SmartDashboard.putNumber("Right Side Velocity", getRightSideVelocity());

    //driveHeadingWidget.setDouble(getHeading());
    SmartDashboard.putNumber("Drive Heading", getHeading());

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  /**
   * Drives the robot at given x, y and theta speeds. Speeds range from [-1, 1]
   * and the linear
   * speeds have no effect on the angular speed.
   *
   * @param forwardSpeed  Speed of the robot moving in the x and y directions
   *                      direction (left/right/forwards/bacwards).
   * @param turnSpeed     Speed of the robot rotating.
   */

  public void ArcadeDrive(double forwardSpeed, double turnSpeed) {

    robotDrive.arcadeDrive(-forwardSpeed, -turnSpeed);

  }

  public void PIDArcadeDrive(double forwardSpeed, double turnSpeed) {
    WheelSpeeds speeds = DifferentialDrive.arcadeDriveIK(-forwardSpeed, -turnSpeed, true);

    leftPIDFront.setReference(speeds.left, ControlType.kVelocity);
    leftPIDBack.setReference(speeds.left, ControlType.kVelocity);
    rightPIDFront.setReference(-speeds.right, ControlType.kVelocity);
    rightPIDBack.setReference(-speeds.right, ControlType.kVelocity);

    robotDrive.feed();
  }

  public void curvatureDrive(double forwardSpeed, double turnSpeed) {

    robotDrive.curvatureDrive(-forwardSpeed, -turnSpeed, (Math.abs(forwardSpeed)) < .05);

  }

  public Pose2d updateOdometry() {

    return m_odometry.update(
      Rotation2d.fromDegrees(getHeading()), 
      getLeftSideDistance(), 
      getRightSideDistance());

  }

  public Pose2d getPose() {

    return m_odometry.getPoseMeters();

  }

  public void resetOdometry(Pose2d position) {

    resetEncoders();

    m_odometry.resetPosition(
      Rotation2d.fromDegrees(getHeading()), 
      getLeftSideDistance(), 
      getRightSideDistance(), 
      position);

  }

  public void setField2d(PathPlannerTrajectory trajectory) {

    m_field2d.getObject("Trajectory").setTrajectory(trajectory);

  }

  public void resetEncoders() {

    leftEncoderFront.setPosition(0);
    leftEncoderBack.setPosition(0);
    rightEncoderFront.setPosition(0);
    rightEncoderBack.setPosition(0);

  }

  public double getAverageEncoderDistance() {
    return (getLeftSideDistance() + getRightSideDistance()) / 2.0;
  }

  public double getLeftSideDistance() {
    return (leftEncoderFront.getPosition() + leftEncoderBack.getPosition());
  }

  public double getRightSideDistance() {
    return -(rightEncoderFront.getPosition() + rightEncoderBack.getPosition());
  }

  public double getLeftSideVelocity() {
    return (leftEncoderFront.getVelocity() + leftEncoderBack.getVelocity());
  }

  public double getRightSideVelocity() {
    return -(rightEncoderFront.getVelocity() + rightEncoderBack.getVelocity());
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
      leftEncoderFront.getVelocity() + leftEncoderBack.getVelocity(), 
      -rightEncoderFront.getVelocity() + -rightEncoderBack.getVelocity());
  }

  public double getHeading() {

    return -navXGyro.getYaw() + 180;

  }

  public Rotation2d getRotation2d() {

    return navXGyro.getRotation2d();
    
  }

  public double getPitch() {

    return navXGyro.getPitch();

  }

  public double getTurnRate() {

    return navXGyro.getRate();

  }

  public void setPIDF(double kP, double kI, double kD, double kF) {
    leftPIDFront.setP(kP);
    leftPIDBack.setP(kP);
    rightPIDFront.setP(kP);
    rightPIDBack.setP(kP);

    leftPIDFront.setI(kI);
    leftPIDBack.setI(kI);
    rightPIDFront.setI(kI);
    rightPIDBack.setI(kI);

    leftPIDFront.setD(kD);
    leftPIDBack.setD(kD);
    rightPIDFront.setD(kD);
    rightPIDBack.setD(kD);

    leftPIDFront.setFF(kF);
    leftPIDBack.setFF(kF);
    rightPIDFront.setFF(kF);
    rightPIDBack.setFF(kF);
  }

  public void setMaxSpeed(double max) {
    leftPIDFront.setOutputRange(-max, max);
    leftPIDBack.setOutputRange(-max, max);
    rightPIDFront.setOutputRange(-max, max);
    rightPIDBack.setOutputRange(-max, max);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {

    leftMotors.setVoltage(leftVolts);
    rightMotors.setVoltage(rightVolts);

    robotDrive.feed();

  }

  public void stopDrive() {

    tankDriveVolts(0, 0);

  }

}
