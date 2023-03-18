/// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.math.controller.ProfiledPIDController;
//import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
//import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;

import java.util.Map;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.revrobotics.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.swervedrivespecialties.swervelib.*;
import com.swervedrivespecialties.swervelib.SwerveModule;

public class SwerveDrivetrain extends SubsystemBase {
  
  //Creates drivetrain motor objects and groups
  private final CANSparkMax 
  leftMotorFrontDrive, 
  leftMotorFrontSteer, 
  leftMotorBackDrive, 
  leftMotorBackSteer, 
  rightMotorFrontDrive, 
  rightMotorFrontSteer, 
  rightMotorBackDrive, 
  rightMotorBackSteer;

  private final RelativeEncoder 
  leftEncoderFrontDrive, 
  leftEncoderFrontSteer, 
  leftEncoderBackDrive, 
  leftEncoderBackSteer, 
  rightEncoderFrontDrive, 
  rightEncoderFrontSteer, 
  rightEncoderBackDrive, 
  rightEncoderBackSteer;

  private final SwerveModule leftModuleFront, leftModuleBack, rightModuleFront, rightModuleBack;

  //private final PIDController m_drivePIDController;
  //private final ProfiledPIDController m_steerPIDController;

  //private final SimpleMotorFeedforward m_driveFeedForward;
  //private final SimpleMotorFeedforward m_steerFeedForward;

  private final SwerveDriveOdometry m_swerveDrivetrainOdometry;

  private final AHRS navXGyro;

  private final Field2d m_field2d;

  private final Timer autoTrajectoryTimer;

  private PathPlannerTrajectory currentAutoTrajectory;

  //public double leftFrontLastPosition, leftBackLastPosition, rightFrontLastPosition, rightBackLastPosition;
  // The current target position of every motor
  //public double leftTargetPositionFront, leftTargetPositionBack, rightTargetPositionFront, rightTargetPositionBack;

  private final ShuffleboardTab teleopTab;

  private final GenericEntry driveHeadingWidget, /*drivePitchWidget,*/driveRollWidget;

  /** Creates a new Drivetrain. */
  public SwerveDrivetrain() {

    leftMotorFrontDrive = new CANSparkMax(Constants.Drivetrain.LEFT_FRONT_PORT_DRIVE, MotorType.kBrushless);
    leftMotorFrontSteer = new CANSparkMax(Constants.Drivetrain.LEFT_FRONT_PORT_STEER, MotorType.kBrushless);

    leftMotorBackDrive = new CANSparkMax(Constants.Drivetrain.LEFT_BACK_PORT_DRIVE, MotorType.kBrushless);
    leftMotorBackSteer = new CANSparkMax(Constants.Drivetrain.LEFT_BACK_PORT_STEER, MotorType.kBrushless);

    rightMotorFrontDrive = new CANSparkMax(Constants.Drivetrain.RIGHT_FRONT_PORT_DRIVE, MotorType.kBrushless);
    rightMotorFrontSteer = new CANSparkMax(Constants.Drivetrain.RIGHT_FRONT_PORT_STEER, MotorType.kBrushless);

    rightMotorBackDrive = new CANSparkMax(Constants.Drivetrain.RIGHT_BACK_PORT_DRIVE, MotorType.kBrushless);
    rightMotorBackSteer = new CANSparkMax(Constants.Drivetrain.RIGHT_BACK_PORT_STEER, MotorType.kBrushless);

    leftEncoderFrontDrive = createEncoder(leftMotorFrontDrive, true);
    leftEncoderFrontSteer = createEncoder(leftMotorFrontSteer, false);

    leftEncoderBackDrive = createEncoder(leftMotorBackDrive, true);
    leftEncoderBackSteer = createEncoder(leftMotorBackSteer, false);

    rightEncoderFrontDrive = createEncoder(rightMotorFrontDrive, true);
    rightEncoderFrontSteer = createEncoder(rightMotorFrontSteer, false);

    rightEncoderBackDrive = createEncoder(rightMotorBackDrive, true);
    rightEncoderBackSteer = createEncoder(rightMotorBackSteer, false);


    //The gear ratios, motor ports, and steer offsets for these object declarations are placholders for now
    leftModuleFront = Mk4iSwerveModuleHelper.createNeo(
      Shuffleboard.getTab("SmartDashboard").getLayout("Left Front Module", BuiltInLayouts.kList), 
      Mk4iSwerveModuleHelper.GearRatio.L1, 
      Constants.Drivetrain.LEFT_FRONT_PORT_DRIVE, 
      Constants.Drivetrain.LEFT_FRONT_PORT_STEER, 
      Constants.Drivetrain.LEFT_FRONT_PORT_STEER, 
      0);

    leftModuleBack = Mk4iSwerveModuleHelper.createNeo(
      Shuffleboard.getTab("SmartDashboard").getLayout("Left Back Module", BuiltInLayouts.kList), 
      Mk4iSwerveModuleHelper.GearRatio.L1, 
      Constants.Drivetrain.LEFT_BACK_PORT_DRIVE, 
      Constants.Drivetrain.LEFT_BACK_PORT_STEER, 
      Constants.Drivetrain.LEFT_BACK_PORT_STEER, 
      0);

    rightModuleFront = Mk4iSwerveModuleHelper.createNeo(
      Shuffleboard.getTab("SmartDashboard").getLayout("Right Front Module", BuiltInLayouts.kList), 
      Mk4iSwerveModuleHelper.GearRatio.L1, 
      Constants.Drivetrain.RIGHT_FRONT_PORT_DRIVE, 
      Constants.Drivetrain.RIGHT_FRONT_PORT_STEER, 
      Constants.Drivetrain.RIGHT_FRONT_PORT_STEER, 
      0);

    rightModuleBack = Mk4iSwerveModuleHelper.createNeo(
      Shuffleboard.getTab("SmartDashboard").getLayout("Right Back Module", BuiltInLayouts.kList), 
      Mk4iSwerveModuleHelper.GearRatio.L1, 
      Constants.Drivetrain.RIGHT_BACK_PORT_DRIVE, 
      Constants.Drivetrain.RIGHT_BACK_PORT_STEER, 
      Constants.Drivetrain.RIGHT_BACK_PORT_STEER, 
      0);

    navXGyro = new AHRS(SerialPort.Port.kUSB1);

    m_field2d = new Field2d();

    autoTrajectoryTimer = new Timer();

    currentAutoTrajectory = new PathPlannerTrajectory();

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

    // Creates and pushes Field2d to SmartDashboard.
    SmartDashboard.putData(m_field2d); 

    //These standard deviation values should be measured proplerly for our robot
    m_swerveDrivetrainOdometry = new SwerveDriveOdometry(
      Constants.Drivetrain.kSwerveKinematics, 
      getRotation2d(), 
      getPositions(),
      new Pose2d(8.28, 4, Rotation2d.fromDegrees(0)));

    // These gain values are placeholders for now
    //m_driveFeedForward = new SimpleMotorFeedforward(1, 3);
    //m_steerFeedForward = new SimpleMotorFeedforward(1, 0.5);

    //m_drivePIDController = new PIDController(Constants.kPDriveVel, 0, 0);

    // Using a TrapezoidProfile PIDController to allow for smooth turning
    /*m_steerPIDController = new ProfiledPIDController(
        Constants.kPDriveVel,
        0,
        0,
        new TrapezoidProfile.Constraints(
          Constants.kMaxAngularVelocityRadiansPerSecond,
          Constants.kMaxAngularAccelerationRadiansPerSecondSquared));

    m_steerPIDController.enableContinuousInput(-Math.PI, Math.PI);*/

  }

  @Override
  public void periodic() {
  // This method will be called once per scheduler run

    updateOdometry();

    m_field2d.setRobotPose(getPose());

    driveHeadingWidget.setDouble(getHeading());
    //drivePitchWidget.setDouble(getPitch());
    driveRollWidget.setDouble(getRoll());

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public RelativeEncoder createEncoder(CANSparkMax motor, boolean isDriveEncoder) {

    RelativeEncoder encoder = motor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, Constants.Encoder.CPR);

    if(isDriveEncoder) {

      encoder.setVelocityConversionFactor(Constants.Drivetrain.LINEAR_VELOCITY_CONVERSION_FACTOR);
      encoder.setPositionConversionFactor(Constants.Drivetrain.DISTANCE_CONVERSION_FACTOR);

    }
    else {

      encoder.setVelocityConversionFactor(Constants.Drivetrain.ANGULAR_VELOCITY_CONVERSION_FACTOR);
      encoder.setPositionConversionFactor(Constants.Drivetrain.ANGLE_CONVERSION_FACTOR);

    }

    return encoder;

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

      leftMotorFrontDrive.setIdleMode(IdleMode.kBrake);
      leftMotorFrontSteer.setIdleMode(IdleMode.kBrake);
      leftMotorBackDrive.setIdleMode(IdleMode.kBrake);
      leftMotorBackSteer.setIdleMode(IdleMode.kBrake);
      rightMotorFrontDrive.setIdleMode(IdleMode.kBrake);
      rightMotorFrontSteer.setIdleMode(IdleMode.kBrake);
      rightMotorBackDrive.setIdleMode(IdleMode.kBrake);
      rightMotorBackSteer.setIdleMode(IdleMode.kBrake);

    }
    else {
      
      leftMotorFrontDrive.setIdleMode(IdleMode.kCoast);
      leftMotorFrontSteer.setIdleMode(IdleMode.kCoast);
      leftMotorBackDrive.setIdleMode(IdleMode.kCoast);
      leftMotorBackSteer.setIdleMode(IdleMode.kCoast);
      rightMotorFrontDrive.setIdleMode(IdleMode.kCoast);
      rightMotorFrontSteer.setIdleMode(IdleMode.kCoast);
      rightMotorBackDrive.setIdleMode(IdleMode.kCoast);
      rightMotorBackSteer.setIdleMode(IdleMode.kCoast);

    }

  }

  public void setField2d(PathPlannerTrajectory trajectory) {

    // Pushes the trajectory to Field2d.
    m_field2d.getObject("Trajectory").setTrajectory(trajectory);

  }

  public void startAutoTrajectoryTimer() {

    autoTrajectoryTimer.reset();
    autoTrajectoryTimer.start();

  }

  public void setCurrentAutoTrajectory(PathPlannerTrajectory trajectory) {

    currentAutoTrajectory = trajectory;

  }

  public Rotation2d getAutoTrajectoryRotation() {

    return currentAutoTrajectory.sample(autoTrajectoryTimer.get()).poseMeters.getRotation();

  }

  public Pose2d updateOdometry() {

    return m_swerveDrivetrainOdometry.update(getRotation2d(), getPositions());

  }

  public SwerveModulePosition[] getPositions() {

    SwerveModulePosition[] positions = {
      new SwerveModulePosition(
        leftEncoderFrontDrive.getPosition(), 
        Rotation2d.fromRadians(leftEncoderFrontSteer.getPosition())),
      new SwerveModulePosition(
        leftEncoderBackDrive.getPosition(), 
        Rotation2d.fromRadians(leftEncoderBackSteer.getPosition())),
      new SwerveModulePosition(
        rightEncoderFrontDrive.getPosition(), 
        Rotation2d.fromRadians(rightEncoderFrontSteer.getPosition())),
      new SwerveModulePosition(
        rightEncoderBackDrive.getPosition(), 
        Rotation2d.fromRadians(rightEncoderBackSteer.getPosition()))};

    return positions;

  }

  public Pose2d getPose() {
    return m_swerveDrivetrainOdometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d position) {

    m_swerveDrivetrainOdometry.resetPosition(getRotation2d(), getPositions(), position);

  }

  /*public void setDesiredState(SwerveModuleState desiredState, CANSparkMax driveMotor, CANSparkMax steerMotor, RelativeEncoder driveEncoder, RelativeEncoder steerEncoder) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, new Rotation2d(Units.rotationsToRadians(steerEncoder.getPosition())));

    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        m_drivePIDController.calculate(driveEncoder.getVelocity(), state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput =
        m_steerPIDController.calculate(steerEncoder.getPosition(), state.angle.getRadians());

    final double driveFeedForward = m_driveFeedForward.calculate(state.speedMetersPerSecond);

    final double turnFeedForward = m_steerFeedForward.calculate(m_steerPIDController.getSetpoint().velocity);

    driveMotor.setVoltage(driveOutput + driveFeedForward);
    steerMotor.setVoltage(turnOutput + turnFeedForward);

  }

  public void swerveDrive(double translationX, double translationY, double rotation) {

    SwerveModuleState[] states = Constants.kSwerveKinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(
      translationX, 
      translationY, 
      rotation, 
      getRotation2d()));

    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.kMaxSpeedMetersPerSeconds);

    //The maximum voltage value of 10 is taken from Trajectories
    setDesiredState(states[0], leftMotorFrontDrive, leftMotorFrontSteer, leftEncoderFrontDrive, leftEncoderFrontSteer);
    setDesiredState(states[1], leftMotorBackDrive, leftMotorBackSteer, leftEncoderBackDrive, leftEncoderBackSteer);
    setDesiredState(states[2], rightMotorFrontDrive, rightMotorFrontSteer, rightEncoderFrontDrive, rightEncoderFrontSteer);
    setDesiredState(states[3], rightMotorBackDrive, rightMotorBackSteer, rightEncoderBackDrive, rightEncoderBackSteer);

  }*/

  public void swerveDrive(double translationX, double translationY, double rotation) {

    SwerveModuleState[] states = Constants.Drivetrain.kSwerveKinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(
      translationX, 
      translationY, 
      rotation, 
      getRotation2d()));

    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Drivetrain.kMaxSpeedMetersPerSeconds);

    //The number that the max speed is multiplyed by is the maxiumum voltage, which is taken from Trajectories
    leftModuleFront.set(states[0].speedMetersPerSecond / Constants.Drivetrain.kMaxSpeedMetersPerSeconds * 10, states[0].angle.getRadians());
    leftModuleBack.set(states[1].speedMetersPerSecond / Constants.Drivetrain.kMaxSpeedMetersPerSeconds * 10, states[1].angle.getRadians());
    rightModuleFront.set(states[2].speedMetersPerSecond / Constants.Drivetrain.kMaxSpeedMetersPerSeconds * 10, states[2].angle.getRadians());
    rightModuleBack.set(states[3].speedMetersPerSecond / Constants.Drivetrain.kMaxSpeedMetersPerSeconds * 10, states[3].angle.getRadians());

  }
  
  public double getHeading() {

    return -navXGyro.getYaw() + 180;

  }

  public double getPitch() {

    return navXGyro.getPitch() + 1;

  }

  public double getRoll() {

    return navXGyro.getRoll() + 1.7;

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

  /*public void autoOutputModuleStates(SwerveModuleState[] states) {

    //The maximum voltage value of 10 is taken from Trajectories
    setDesiredState(states[0], leftMotorFrontSteer, leftMotorFrontDrive, leftEncoderFrontDrive, leftEncoderFrontSteer);
    setDesiredState(states[0], leftMotorBackSteer, leftMotorBackDrive, leftEncoderBackDrive, leftEncoderBackSteer);
    setDesiredState(states[0], rightMotorFrontSteer, rightMotorFrontDrive, rightEncoderFrontDrive, rightEncoderFrontSteer);
    setDesiredState(states[0], rightMotorBackSteer, rightMotorBackDrive, rightEncoderBackDrive, rightEncoderBackSteer);

  }*/

  public void autoOutputModuleStates(SwerveModuleState[] states) {

    //The maximum voltage value of 10 is taken from Trajectories
    leftModuleFront.set(states[0].speedMetersPerSecond / Constants.Drivetrain.kMaxSpeedMetersPerSeconds * 10, states[0].angle.getRadians());
    leftModuleBack.set(states[1].speedMetersPerSecond / Constants.Drivetrain.kMaxSpeedMetersPerSeconds * 10, states[1].angle.getRadians());
    rightModuleFront.set(states[2].speedMetersPerSecond / Constants.Drivetrain.kMaxSpeedMetersPerSeconds * 10, states[2].angle.getRadians());
    rightModuleBack.set(states[2].speedMetersPerSecond / Constants.Drivetrain.kMaxSpeedMetersPerSeconds * 10, states[2].angle.getRadians());

  }

  public void stopDrive() {

    SwerveModuleState[] stopStates = {
      new SwerveModuleState(0, new Rotation2d(Units.rotationsToRadians(leftEncoderFrontSteer.getPosition()))),
      new SwerveModuleState(0, new Rotation2d(Units.rotationsToRadians(leftEncoderBackSteer.getPosition()))),
      new SwerveModuleState(0, new Rotation2d(Units.rotationsToRadians(rightEncoderFrontSteer.getPosition()))),
      new SwerveModuleState(0, new Rotation2d(Units.rotationsToRadians(rightEncoderBackSteer.getPosition())))};

    autoOutputModuleStates(stopStates);
    
  }


}
