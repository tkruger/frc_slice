/// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
//import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.math.geometry.Pose2d;
//import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
//import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.StateSpaceUtil;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
//import edu.wpi.first.wpilibj.Encoder;
//import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.math.VecBuilder;
/*import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;*/
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.util.Units;

public class Drivetrain extends SubsystemBase {
  
  //Creates drivetrain motor objects and groups
  private final CANSparkMax leftMotorFront, leftMotorBack, rightMotorFront, rightMotorBack;
  private final MotorControllerGroup leftMotors, rightMotors;
  private final DifferentialDrive robotDrive;

  private final ADXRS450_Gyro m_gyro;

  //private final DifferentialDriveOdometry m_drivetrainOdometry;

  public final RelativeEncoder leftEncoderFront, leftEncoderBack, rightEncoderFront, rightEncoderBack;
  
  // PID Controllers
  public final SparkMaxPIDController leftPIDFront, leftPIDBack, rightPIDFront, rightPIDBack;

  public static DifferentialDrivePoseEstimator m_poseEstimator;

  public final Field2d field2d = new Field2d();

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

    //(NOTE TO SELF) Check if we use quadature encoders
    leftEncoderFront = leftMotorFront.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, Constants.drivetrain_ENCODER_CPR);
    leftEncoderBack = leftMotorBack.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, Constants.drivetrain_ENCODER_CPR);

    rightEncoderFront = rightMotorFront.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, Constants.drivetrain_ENCODER_CPR);
    rightEncoderBack = rightMotorBack.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, Constants.drivetrain_ENCODER_CPR);

    leftPIDFront = leftMotorFront.getPIDController();
    leftPIDBack = leftMotorBack.getPIDController();
    rightPIDFront = rightMotorFront.getPIDController();
    rightPIDBack = rightMotorBack.getPIDController();

    m_gyro = new ADXRS450_Gyro();

    //m_drivetrainOdometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());

    //These standard deviation values should be measured proplerly for our robot
    m_poseEstimator = new DifferentialDrivePoseEstimator(m_gyro.getRotation2d(),
      new Pose2d(),
      VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(0.1), 0.1, 0.1),
      VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(0.1)),
      VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(0.1)));

    // Display current gyro heading on Shuffleboard
    Shuffleboard.getTab("SmartDashboard").add(m_gyro);
    // Display how the robot is moving on Shuffleboard
    Shuffleboard.getTab("SmartDashboard").add(robotDrive);


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    field2d.setRobotPose(getEstimatedPosition());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public Pose2d getEstimatedGlobalPose(Pose2d estimatedRobotPose) {

    //These white noise element intensity values should be proplery measured for our robot
    var rand = 
      StateSpaceUtil.makeWhiteNoiseVector(VecBuilder.fill(0, 0, Units.degreesToRadians(0)));

    return new Pose2d(
        estimatedRobotPose.getX() + rand.get(0, 0),
        estimatedRobotPose.getY() + rand.get(1, 0),
        estimatedRobotPose.getRotation().plus(new Rotation2d(rand.get(2, 0))));

  }

  public Pose2d updateOdometry() {

    m_poseEstimator.update(
      m_gyro.getRotation2d(), 
        new DifferentialDriveWheelSpeeds(
          getAverageLeftEncoderVelocity(), 
          getAverageRightEncoderVelocity()),
      getAverageLeftEncoderDistance(),
      getAverageRightEncoderDistance());

    //This latency value(0.3) is a place holder for now and should be measured properly for our robot
    // m_poseEstimator.addVisionMeasurement(
    //   getEstimatedGlobalPose(m_poseEstimator.getEstimatedPosition()),
    //   Timer.getFPGATimestamp() - 0.3);

    return m_poseEstimator.getEstimatedPosition();

  }

  public Pose2d getEstimatedPosition() {
    return m_poseEstimator.getEstimatedPosition();
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

    robotDrive.arcadeDrive(-forwardSpeed, turnSpeed);

  }

  public void PIDArcadeDrive(double forwardSpeed, double turnSpeed) {
    WheelSpeeds speeds = DifferentialDrive.arcadeDriveIK(-forwardSpeed, turnSpeed, true);

    leftPIDFront.setReference(speeds.left, ControlType.kVelocity);
    leftPIDBack.setReference(speeds.left, ControlType.kVelocity);
    rightPIDFront.setReference(speeds.right, ControlType.kVelocity);
    rightPIDBack.setReference(speeds.right, ControlType.kVelocity);
  }

  public void resetOdometry(Pose2d position) {

    leftEncoderFront.setPosition(0);
    leftEncoderBack.setPosition(0);
    rightEncoderFront.setPosition(0);
    rightEncoderBack.setPosition(0);

    m_poseEstimator.resetPosition(position, m_gyro.getRotation2d());

    m_gyro.reset();

  }

  public double getLeftFrontVelocity() {

    return
    leftEncoderFront.getVelocity();

  }

  public double getLeftBackVelocity() {

    return
    leftEncoderBack.getVelocity();

  }

  public double getRightFrontVelocity() {

    return
    rightEncoderFront.getVelocity();

  }

  public double getRightBackVelocity() {

    return
    rightEncoderBack.getVelocity();

  }

  public double getAverageEncoderDistance() {
    return (getAverageLeftEncoderDistance() + getAverageRightEncoderDistance()) / 2.0;
  }

  public double getAverageLeftEncoderDistance() {
      return (leftEncoderFront.getPosition() + leftEncoderBack.getPosition()) / 2.0; 
  }

  public double getAverageRightEncoderDistance() {
    return (rightEncoderFront.getPosition() + rightEncoderBack.getPosition()) / 2.0; 
  }

  public double getAverageLeftEncoderVelocity() {
    return (leftEncoderFront.getVelocity() + leftEncoderBack.getVelocity()) / 2.0; 
  }

  public double getAverageRightEncoderVelocity() {
  return (rightEncoderFront.getVelocity() + rightEncoderBack.getVelocity()) / 2.0; 
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getAverageLeftEncoderVelocity(), getAverageRightEncoderVelocity());
  }

  
  public double getHeading() {

    return m_gyro.getRotation2d().getDegrees();

  }

  public double getTurnRate() {

    return -m_gyro.getRate();

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

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMotors.setVoltage(leftVolts);
    rightMotors.setVoltage(rightVolts);
    robotDrive.feed();
  }

  public void stopDrive() {
    tankDriveVolts(0, 0);
  }

}

