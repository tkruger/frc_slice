/// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import frc.robot.*;
import frc.robot.factories.SparkMaxFactory;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
//import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;

import com.revrobotics.*;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.AHRS.SerialDataType;

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

  //private final DifferentialDriveOdometry m_odometry;
  private final DifferentialDrivePoseEstimator m_odometry;

  private final ShuffleboardTab teleopTab;

  private final GenericEntry 
  //leftSidePositionWidget, 
  //rightSidePositionWidget, 
  //leftSideVelocityWidget, 
  //rightSideVelocityWidget, 
  driveHeadingWidget, 
  //drivePitchWidget,
  driveRollWidget;

  private final Field2d m_field2d;

  private final Timer m_timer;

  private Pose2d botPose;

  private boolean forceVisionImplementation = false;

  private boolean drivetrainReversed = false;

  private final UsbCamera cameraFeed;

  /** Creates a new Drivetrain. */
  public Drivetrain() {

    m_timer = new Timer();

    m_timer.reset();
    m_timer.start();

    // Instantiates motors and motor groups
    leftMotorFront = SparkMaxFactory.createDefaultSparkMax(Constants.Drivetrain.LEFT_FRONT_PORT);
    leftMotorBack = SparkMaxFactory.createDefaultSparkMax(Constants.Drivetrain.LEFT_BACK_PORT);
    rightMotorFront = SparkMaxFactory.createDefaultSparkMax(Constants.Drivetrain.RIGHT_FRONT_PORT);
    rightMotorBack = SparkMaxFactory.createDefaultSparkMax(Constants.Drivetrain.RIGHT_BACK_PORT);

    leftMotors = new MotorControllerGroup(leftMotorFront, leftMotorBack);
    rightMotors = new MotorControllerGroup(rightMotorFront, rightMotorBack);

    rightMotors.setInverted(true);

    robotDrive = new DifferentialDrive(leftMotors, rightMotors);

    leftEncoderFront = leftMotorFront.getEncoder(
      SparkMaxRelativeEncoder.Type.kHallSensor,
      Constants.Encoder.CPR);
    leftEncoderBack = leftMotorBack.getEncoder(
      SparkMaxRelativeEncoder.Type.kHallSensor,
      Constants.Encoder.CPR);
    rightEncoderFront = rightMotorFront.getEncoder(
      SparkMaxRelativeEncoder.Type.kHallSensor,
      Constants.Encoder.CPR);
    rightEncoderBack = rightMotorBack.getEncoder(
      SparkMaxRelativeEncoder.Type.kHallSensor,
      Constants.Encoder.CPR);

    leftEncoderFront.setVelocityConversionFactor(Constants.Drivetrain.VELOCITY_CONVERSION_FACTOR);
    leftEncoderBack.setVelocityConversionFactor(Constants.Drivetrain.VELOCITY_CONVERSION_FACTOR);
    rightEncoderFront.setVelocityConversionFactor(Constants.Drivetrain.VELOCITY_CONVERSION_FACTOR);
    rightEncoderBack.setVelocityConversionFactor(Constants.Drivetrain.VELOCITY_CONVERSION_FACTOR);

    leftEncoderFront.setPositionConversionFactor(Constants.Drivetrain.POSITION_CONVERSION_FACTOR);
    leftEncoderBack.setPositionConversionFactor(Constants.Drivetrain.POSITION_CONVERSION_FACTOR);
    rightEncoderFront.setPositionConversionFactor(Constants.Drivetrain.POSITION_CONVERSION_FACTOR);
    rightEncoderBack.setPositionConversionFactor(Constants.Drivetrain.POSITION_CONVERSION_FACTOR);

    leftPIDFront = leftMotorFront.getPIDController();
    leftPIDBack = leftMotorBack.getPIDController();
    rightPIDFront = rightMotorFront.getPIDController();
    rightPIDBack = rightMotorBack.getPIDController();
    
    navXGyro = new AHRS(SerialPort.Port.kUSB1, SerialDataType.kProcessedData, (byte) 60);

    Timer.delay(1.0);

    navXGyro.enableLogging(true);

    m_field2d = new Field2d();

    resetEncoders();

    /*m_odometry = new DifferentialDriveOdometry(
      Rotation2d.fromDegrees(getHeading()), 
      getLeftSideDistance(), 
      getRightSideDistance(),
      new Pose2d(8, 4, getRotation2d()));*/

    m_odometry = new DifferentialDrivePoseEstimator(
      Constants.Autonomous.kDriveKinematics,
      getRotation2d(), 
      getLeftSideDistance(),
      getRightSideDistance(),
      new Pose2d(8, 4, Rotation2d.fromDegrees(0)));

    //Creates the "Driver Tab" on Shuffleboard
    teleopTab = Shuffleboard.getTab("Teleop Tab");

    //Creates a widget for showing the gyro heading
    driveHeadingWidget = 
    teleopTab.add("Drive Heading", 0.0).
    withWidget(BuiltInWidgets.kDial).
    withProperties(Map.of("Min", 0, "Max", 360)).
    withPosition(0, 0).
    withSize(2, 1).
    getEntry();


    /*//Creates a widget for showing the gyro pitch
    drivePitchWidget = 
    teleopTab.add("Drive Pitch", 0.0).
    withWidget(BuiltInWidgets.kDial).
    withProperties(Map.of("Min", -180, "Max", 180)).
    withPosition(7, 0).
    withSize(2, 1).
    getEntry();*/

    //Creates a widget for showing the gyro roll
    driveRollWidget = 
    teleopTab.add("Drive Roll", 0.0).
    withWidget(BuiltInWidgets.kDial).
    withProperties(Map.of("Min", -180, "Max", 180)).
    withPosition(7, 0).
    withSize(2, 1).
    getEntry();

    //Creates a widget for showing the drivetrain left side position
    /*leftSidePositionWidget = 
    driveTab.add("Left Side Position", 0.0).
    withPosition(0, 0).
    withSize(2, 1).
    getEntry();*/

    //Creates a widget for showing the drivetrain right side position
    /*rightSidePositionWidget = 
    driveTab.add("Right Side Position", 0.0).
    withPosition(7, 0).
    withSize(2, 1).
    getEntry();*/

    //Creates a widget for showing the drivetrain left side position
    /*leftSideVelocityWidget = 
    teleopTab.add("Left Side Velocity", 0.0).
    withPosition(3, 2).
    withSize(3, 1).
    getEntry();*/

    //Creates a widget for showing the drivetrain left side position
    /*rightSideVelocityWidget = 
    teleopTab.add("Right Side Velocity", 0.0).
    withPosition(3, 3).
    withSize(3, 1).
    getEntry();*/

    cameraFeed = CameraServer.startAutomaticCapture();

    //Displays how the robot is moving on Shuffleboard
    teleopTab.add(robotDrive).withPosition(6, 2).withSize(3, 2);

    //Displays the current position of the robot on the field on Shuffleboard
    teleopTab.add(m_field2d).withPosition(0, 2).withSize(3, 2);

    //Displays the feed from the USB camera on Shufflboard
    teleopTab.add(cameraFeed).withWidget(BuiltInWidgets.kCameraStream).withPosition(3, 1).withSize(3, 3);

    //Sends the Fiel2d object to NetworkTables
    SmartDashboard.putData(m_field2d);

    drivetrainReversed = false;

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    updateOdometry();

    m_field2d.setRobotPose(getPose());

    //leftSidePositionWidget.setDouble(getLeftSideDistance());
    //rightSidePositionWidget.setDouble(getRightSideDistance());

    //leftSideVelocityWidget.setDouble(getLeftSideVelocity());
    //rightSideVelocityWidget.setDouble(getRightSideVelocity());

    driveHeadingWidget.setDouble(getHeading());
    //drivePitchWidget.setDouble(getPitch());
    driveRollWidget.setDouble(getRoll());

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  /**
   * Sets the idle mode of all drivetrain motors to either brake mode or coast mode.
   * 
   * @param enableBrakeMode Whether or not the idle mode of all 
   * drivetrain motors should be set to brake mode(false to set to coast mode).
   * 
   */
  public void setIdleMode(boolean enableBrakeMode) {

    if(enableBrakeMode) {

      leftMotorFront.setIdleMode(IdleMode.kBrake);
      leftMotorBack.setIdleMode(IdleMode.kBrake);
      rightMotorFront.setIdleMode(IdleMode.kBrake);
      rightMotorBack.setIdleMode(IdleMode.kBrake);

    }
    else {
      
      leftMotorFront.setIdleMode(IdleMode.kCoast);
      leftMotorBack.setIdleMode(IdleMode.kCoast);
      rightMotorFront.setIdleMode(IdleMode.kCoast);
      rightMotorBack.setIdleMode(IdleMode.kCoast);

    }

  }

  /**
   * Reverses the forward speed outputs for all 3 drive methods.
   */
  public void reverseDrivetrain() {

    drivetrainReversed = !drivetrainReversed;

  }

  /**
   * Drives the robot at given x, y and theta speeds. Speeds range from [-1, 1],
   * with the linear speeds having no effect on the angular speed.
   *
   * @param forwardSpeed  Speed of the robot moving in the x and y directions
   *                      direction (left/right/forwards/bacwards).
   * @param turnSpeed     Speed of the robot rotating.
   */

  public void ArcadeDrive(double forwardSpeed, double turnSpeed) {

    if(drivetrainReversed == false) {

      robotDrive.arcadeDrive(-forwardSpeed, -turnSpeed);

    }
    else {

      robotDrive.arcadeDrive(forwardSpeed, -turnSpeed);

    }

  }

  /**
   * Drives the robot using PIDF controllers at given x, y and theta speeds. Speeds range from [-1, 1],
   * with the linear speeds having no effect on the angular speed.
   *
   * @param forwardSpeed  Speed of the robot moving in the x and y directions
   *                      direction (left/right/forwards/bacwards).
   * @param turnSpeed     Speed of the robot rotating.
   */
  public void PIDArcadeDrive(double forwardSpeed, double turnSpeed) {
    WheelSpeeds speeds;

    if(drivetrainReversed == false) {

      speeds = DifferentialDrive.arcadeDriveIK(-forwardSpeed, -turnSpeed, true);

    }
    else {

      speeds = DifferentialDrive.arcadeDriveIK(forwardSpeed, -turnSpeed, true);

    }

    leftPIDFront.setReference(speeds.left, ControlType.kVelocity);
    leftPIDBack.setReference(speeds.left, ControlType.kVelocity);
    rightPIDFront.setReference(-speeds.right, ControlType.kVelocity);
    rightPIDBack.setReference(-speeds.right, ControlType.kVelocity);

    robotDrive.feed();
  }

  /**
   * Drives the robot at given x, y and theta speeds. Speeds range from [-1, 1],
   * with the turn speed controlling the curvature of the robot's path rather than
   * the rate of heading change.
   *
   * @param forwardSpeed  Speed of the robot moving in the x and y directions
   *                      direction (left/right/forwards/bacwards).
   * @param turnSpeed     Speed of the robot rotating.
   */
  public void curvatureDrive(double forwardSpeed, double turnSpeed) {

    if(drivetrainReversed == false) {

      robotDrive.curvatureDrive(-forwardSpeed, -turnSpeed, (Math.abs(forwardSpeed)) < .05);

    }
    else {

      robotDrive.curvatureDrive(forwardSpeed, -turnSpeed, (Math.abs(forwardSpeed)) < .05);

    }

  }

  public void toggleVisionImplementation() {

    forceVisionImplementation = !forceVisionImplementation;

  }

  /*public Pose2d updateOdometry() {

    return m_odometry.update(
      getRotation2d(), 
      getLeftSideDistance(), 
      getRightSideDistance());

  }*/

  /**
   * Updates the drivetrain odometry object,
   * adding a vision meausurement from the limelight
   * if it gives a pose that is within a certain range of the main pose.
   * 
   * @return The new updated pose of the robot.
   */
  public Pose2d updateOdometry() {

    m_odometry.update(
      getRotation2d(), 
      getLeftSideDistance(), 
      getRightSideDistance());

      botPose = Limelight.getBotPoseBlue();

      if(botPose != null && ((Math.abs(botPose.getX() - getPose().getX()) <= 1 && Math.abs(botPose.getY() - getPose().getY()) <= 1) || forceVisionImplementation)) {

        m_odometry.addVisionMeasurement(botPose, Timer.getFPGATimestamp());
  
      }

    return m_odometry.getEstimatedPosition();

  }

  /*public Pose2d getPose() {

    return m_odometry.getPoseMeters();

  }*/

  /**
   * Returns the current pose of the robot without updating
   * the odometry.
   * 
   * @return The current estimated pose of the robot.
   */
  public Pose2d getPose() {

    return m_odometry.getEstimatedPosition();

  }

  /**
   * Resets the position of the odometry object using a specified position.
   * 
   * @param position The desired position to reset the odometry of the robot to.
   */
  public void resetOdometry(Pose2d position) {

    resetEncoders();

    m_odometry.resetPosition(
      getRotation2d(), 
      getLeftSideDistance(), 
      getRightSideDistance(), 
      position);

  }

  /**
   * Sends the poses of a desired trajectory to the Field2d object.
   * 
   * @param trajectory The desired trajectory to send to the Field2d object.
   */
  public void setField2d(PathPlannerTrajectory trajectory) {

    m_field2d.getObject("Trajectory").setTrajectory(trajectory);

  }

  /**
   * Sets the positions of all 4 drivetrain encoders to 0(meters).
   */
  public void resetEncoders() {

    leftEncoderFront.setPosition(0);
    leftEncoderBack.setPosition(0);
    rightEncoderFront.setPosition(0);
    rightEncoderBack.setPosition(0);

  }

  /**
   * Calculates and returns the average of the left and right encoder distances(meters).
   * 
   * @return The average of the left and right encoder distances(meters).
   */
  public double getAverageEncoderDistance() {
    return (getLeftSideDistance() + getRightSideDistance()) / 2.0;
  }

  /**
   * Calculates and returns the sum of both left encoder distances(meters).
   * 
   * @return The sum of both left encoder distances(meters).
   */
  public double getLeftSideDistance() {
    return (leftEncoderFront.getPosition() + leftEncoderBack.getPosition());
  }

  /**
   * Calculates and returns the opposite of the sum of both right encoder distances(meters).
   * 
   * @return The opposite of the sum of both right encoder distances(meters).
   */
  public double getRightSideDistance() {
    return -(rightEncoderFront.getPosition() + rightEncoderBack.getPosition());
  }

  public double getAverageDistance() {
    return (getLeftSideDistance() + getRightSideDistance()) / 2;
  }

  /**
   * Calculates and returns the sum of both left encoder velocities(meters/second).
   * 
   * @return The sum of both left encoder velocities(meters/second).
   */
  public double getLeftSideVelocity() {
    return leftEncoderFront.getVelocity() + leftEncoderBack.getVelocity();
  }

  /**
   * Calculates and returns the opposite of the sum of both right encoder velocities(meters/second).
   * 
   * @return The opposite of the sum of both right encoder velocities(meters/second).
   */
  public double getRightSideVelocity() {
    return -(rightEncoderFront.getVelocity() + rightEncoderBack.getVelocity());
  }

  /**
   * Constructs and returns an instance of the DifferentialDriveWheelSpeeds class 
   * using the left and right side velocities.
   * 
   * @return An instance of the DifferentialDriveWheelSpeeds class 
   *         using the left and right side velocities.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
      getLeftSideVelocity(), 
      getRightSideVelocity());
  }

  /**
   * Obtains and returns the current heading of the robot as a Rotation2d from the gyro object.
   * 
   * @return The current heading of the robot as a Rotation2d.
   */
  public Rotation2d getRotation2d() {

    return navXGyro.getRotation2d();
    
  }

  /**
   * Resets the gyro yaw axis to a heading of 0.
   */
  public void resetHeading() {

    navXGyro.reset();

  }

  /**
   * Obtains and returns the current heading of the robot going positive counter-clockwise from 0 to 360 degrees from the gyro object.
   *
   * @return The current heading of the robot going counter-clockwise positive from 0 to 360 degrees.
   */
  public double getHeading() {

    return -navXGyro.getYaw() + 180;

  }

  /**
   * Obtains and returns the current pitch of the robot from -180 to 180 degrees, with an offset of 1 degree from the gyro object.
   * 
   * @return The current pitch of the robot from -180 to 180 degrees, with an offset of 1 degree.
   */
  public double getPitch() {

    return navXGyro.getPitch() + 1;

  }

  public double getRoll() {
    return navXGyro.getRoll() + 1.7;
  }

  /**
   * Obtains and returns the current rate of rotation of the robot along the yaw axis(degrees/second) from the gyro object.
   * 
   * @return The current rate of rotation of the robot along the yaw axis(degrees/second).
   */
  public double getTurnRate() {

    return navXGyro.getRate();

  }

  /**
   * Sets all drivetrain PIDF controllers to specified proportional, integral, derivative, and feed-forward gains.
   * 
   * @param kP The desired proportional gain of all drivetrain PIDF controllers.
   * @param kI The desired integral gain of all drivetrain PIDF controllers.
   * @param kD The desired derivative gain of all drivetrain PIDF controllers.
   * @param kF The desired feed-forward gain of all drivetrain PIDF controllers.
   */
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

  /**
   * Sets the maxiumum and reverse power of all drivetrain PIDF controllers to a specified value.
   * 
   * @param max The desired maximum forward and reverse power to set all drivetrain PIDF controllers to.
   */
  public void setMaxSpeed(double max) {
    leftPIDFront.setOutputRange(-max, max);
    leftPIDBack.setOutputRange(-max, max);
    rightPIDFront.setOutputRange(-max, max);
    rightPIDBack.setOutputRange(-max, max);
  }

  /**
   * Sets the voltages of the left and right drivetrain motor groups to specified values and feeds the motor safety object.
   * 
   * @param leftVolts The desired voltage to set the left drivetrain motors to.
   * @param rightVolts The desired voltage to set the right drievtrian motors to.
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {

    leftMotors.setVoltage(leftVolts);
    rightMotors.setVoltage(rightVolts);

    robotDrive.feed();

  }

  /**
   * Executes the tankDriveVolts method, feeding both of its parameters with 0.
   */
  public void stopDrive() {

    tankDriveVolts(0, 0);

  }

}
