/// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.*;
import frc.robot.modules.*;
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
import edu.wpi.first.math.trajectory.Trajectory;
//import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;

import java.util.Map;

import com.kauailabs.navx.frc.AHRS;

import com.revrobotics.*;
import com.revrobotics.CANSparkMax;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;

public class SwerveDrivetrain extends SubsystemBase {

  private final SparkMaxSwerveModule leftModuleFront, leftModuleBack, rightModuleFront, rightModuleBack;
  private final SwerveModule leftSDSModuleFront, leftSDSModuleBack, rightSDSModuleFront, rightSDSModuleBack;

  //private final PIDController m_drivePIDController;
  //private final ProfiledPIDController m_steerPIDController;

  //private final SimpleMotorFeedforward m_driveFeedForward;
  //private final SimpleMotorFeedforward m_steerFeedForward;

  private final SwerveDriveOdometry m_swerveDrivetrainOdometry;

  private final AHRS navXGyro;

  private final Field2d m_field2d;

  private final Timer autoTrajectoryTimer;

  private Trajectory currentAutoTrajectory;

  //public double leftFrontLastPosition, leftBackLastPosition, rightFrontLastPosition, rightBackLastPosition;
  // The current target position of every motor
  //public double leftTargetPositionFront, leftTargetPositionBack, rightTargetPositionFront, rightTargetPositionBack;

  private final ShuffleboardTab teleopTab;

  private final GenericEntry driveHeadingWidget, /*drivePitchWidget,*/driveRollWidget;

  /** Creates a new Drivetrain. */
  public SwerveDrivetrain() {

    //The gear ratios, motor ports, and steer offsets for these object declarations are placholders for now
    leftSDSModuleFront = Mk4iSwerveModuleHelper.createNeo(
      Shuffleboard.getTab("LiveWindow").getLayout("Left Front Module", BuiltInLayouts.kList), 
      Mk4iSwerveModuleHelper.GearRatio.L1, 
      Constants.Drivetrain.LEFT_FRONT_PORT_DRIVE, 
      Constants.Drivetrain.LEFT_FRONT_PORT_STEER, 
      Constants.Drivetrain.LEFT_FRONT_PORT_STEER, 
      0);

    leftSDSModuleBack = Mk4iSwerveModuleHelper.createNeo(
      Shuffleboard.getTab("LiveWindow").getLayout("Left Back Module", BuiltInLayouts.kList), 
      Mk4iSwerveModuleHelper.GearRatio.L1, 
      Constants.Drivetrain.LEFT_BACK_PORT_DRIVE, 
      Constants.Drivetrain.LEFT_BACK_PORT_STEER, 
      Constants.Drivetrain.LEFT_BACK_PORT_STEER, 
      0);

    rightSDSModuleFront = Mk4iSwerveModuleHelper.createNeo(
      Shuffleboard.getTab("LiveWindow").getLayout("Right Front Module", BuiltInLayouts.kList), 
      Mk4iSwerveModuleHelper.GearRatio.L1, 
      Constants.Drivetrain.RIGHT_FRONT_PORT_DRIVE, 
      Constants.Drivetrain.RIGHT_FRONT_PORT_STEER, 
      Constants.Drivetrain.RIGHT_FRONT_PORT_STEER, 
      0);

    rightSDSModuleBack = Mk4iSwerveModuleHelper.createNeo(
      Shuffleboard.getTab("LiveWindow").getLayout("Right Back Module", BuiltInLayouts.kList), 
      Mk4iSwerveModuleHelper.GearRatio.L1, 
      Constants.Drivetrain.RIGHT_BACK_PORT_DRIVE, 
      Constants.Drivetrain.RIGHT_BACK_PORT_STEER, 
      Constants.Drivetrain.RIGHT_BACK_PORT_STEER, 
      0);

    //The steer offset arguments for these swerve modules are placeholders for now
    leftModuleFront = new SparkMaxSwerveModule(Constants.Drivetrain.LEFT_FRONT_PORT_DRIVE, Constants.Drivetrain.LEFT_FRONT_PORT_STEER, 0);
    leftModuleBack = new SparkMaxSwerveModule(Constants.Drivetrain.LEFT_BACK_PORT_DRIVE, Constants.Drivetrain.LEFT_BACK_PORT_STEER, 0);
    rightModuleFront = new SparkMaxSwerveModule(Constants.Drivetrain.RIGHT_FRONT_PORT_DRIVE, Constants.Drivetrain.RIGHT_FRONT_PORT_STEER, 0);
    rightModuleBack = new SparkMaxSwerveModule(Constants.Drivetrain.RIGHT_BACK_PORT_DRIVE, Constants.Drivetrain.RIGHT_BACK_PORT_STEER, 0);

    navXGyro = new AHRS(SerialPort.Port.kUSB1);

    m_field2d = new Field2d();

    autoTrajectoryTimer = new Timer();

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

    //Displays the current position of the robot on the field on Shuffleboard
    teleopTab.add(m_field2d).withPosition(3, 2).withSize(3, 2);

    // Creates and pushes Field2d to SmartDashboard.
    SmartDashboard.putData(m_field2d); 

    //These standard deviation values should be measured proplerly for our robot
    m_swerveDrivetrainOdometry = new SwerveDriveOdometry(
      Constants.Drivetrain.kSwerveKinematics, 
      getRotation2d(), 
      getPositions(),
      new Pose2d(8.28, 4, Rotation2d.fromDegrees(0)));

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
   * Sets the idle mode of all drive motors to either brake mode or coast mode.
   * 
   * @param enableBrakeMode Whether or not the idle mode of all 
   * drivetrain motors should be set to brake mode(false to set to coast mode).
   * 
   */
  public void setDriveIdleMode(boolean enableBrakeMode) {

    leftModuleFront.setDriveIdleMode(enableBrakeMode);
    leftModuleBack.setDriveIdleMode(enableBrakeMode);
    rightModuleFront.setDriveIdleMode(enableBrakeMode);
    rightModuleBack.setDriveIdleMode(enableBrakeMode);

  }

  /**
   * Sets the idle mode of all steer motors to either brake mode or coast mode.
   * 
   * @param enableBrakeMode Whether or not the idle mode of all 
   * drivetrain motors should be set to brake mode(false to set to coast mode).
   * 
   */
  public void setSteerIdleMode(boolean enableBrakeMode) {

    leftModuleFront.setSteerIdleMode(enableBrakeMode);
    leftModuleBack.setSteerIdleMode(enableBrakeMode);
    rightModuleFront.setSteerIdleMode(enableBrakeMode);
    rightModuleBack.setSteerIdleMode(enableBrakeMode);

  }

  public void setDrivePIDF(double kP, double kI, double kD, double kF) {

    leftModuleFront.setDrivePIDF(kP, kI, kD, kF);
    leftModuleBack.setDrivePIDF(kP, kI, kD, kF);
    rightModuleFront.setDrivePIDF(kP, kI, kD, kF);
    rightModuleBack.setDrivePIDF(kP, kI, kD, kF);

  }

  public void setSteerPIDF(double kP, double kI, double kD, double kF) {

    leftModuleFront.setSteerPIDF(kP, kI, kD, kF);
    leftModuleBack.setSteerPIDF(kP, kI, kD, kF);
    rightModuleFront.setSteerPIDF(kP, kI, kD, kF);
    rightModuleBack.setSteerPIDF(kP, kI, kD, kF);

  }

  public void setMaxDriveOutput(double max) {

    leftModuleFront.setMaxDriveOutput(max);
    leftModuleBack.setMaxDriveOutput(max);
    rightModuleFront.setMaxDriveOutput(max);
    rightModuleBack.setMaxDriveOutput(max);

  }

  public void setMaxSteerOutput(double max) {

    leftModuleFront.setMaxSteerOutput(max);
    leftModuleBack.setMaxSteerOutput(max);
    rightModuleFront.setMaxSteerOutput(max);
    rightModuleBack.setMaxSteerOutput(max);

  }

  public void setField2d(Trajectory trajectory) {

    // Pushes the trajectory to Field2d.
    m_field2d.getObject("Trajectory").setTrajectory(trajectory);

  }

  public void startAutoTrajectoryTimer() {

    autoTrajectoryTimer.reset();
    autoTrajectoryTimer.start();

  }

  public void setCurrentAutoTrajectory(Trajectory trajectory) {

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
      leftModuleFront.getPosition(),
      leftModuleBack.getPosition(),
      rightModuleFront.getPosition(),
      rightModuleBack.getPosition()};

    return positions;

  }

  public Pose2d getPose() {
    return m_swerveDrivetrainOdometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d position) {

    m_swerveDrivetrainOdometry.resetPosition(getRotation2d(), getPositions(), position);

  }

  public void swerveDrivePID(double translationX, double translationY, double rotation) {

    SwerveModuleState[] states = Constants.Drivetrain.kSwerveKinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(
      translationX, 
      translationY, 
      rotation, 
      getRotation2d()));

    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Drivetrain.kMaxSpeedMetersPerSeconds);

    leftModuleFront.setDesiredState(states[0]);
    leftModuleBack.setDesiredState(states[1]);
    rightModuleFront.setDesiredState(states[2]);
    rightModuleBack.setDesiredState(states[3]);

  }

  public void swerveDrive(double translationX, double translationY, double rotation) {

    SwerveModuleState[] states = Constants.Drivetrain.kSwerveKinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(
      translationX, 
      translationY, 
      rotation, 
      getRotation2d()));

    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Drivetrain.kMaxSpeedMetersPerSeconds);

    //The number that the max speed is multiplyed by is the maxiumum voltage, which is taken from Trajectories
    leftSDSModuleFront.set(states[0].speedMetersPerSecond / Constants.Drivetrain.kMaxSpeedMetersPerSeconds * 10, states[0].angle.getRadians());
    leftSDSModuleBack.set(states[1].speedMetersPerSecond / Constants.Drivetrain.kMaxSpeedMetersPerSeconds * 10, states[1].angle.getRadians());
    rightSDSModuleFront.set(states[2].speedMetersPerSecond / Constants.Drivetrain.kMaxSpeedMetersPerSeconds * 10, states[2].angle.getRadians());
    rightSDSModuleBack.set(states[3].speedMetersPerSecond / Constants.Drivetrain.kMaxSpeedMetersPerSeconds * 10, states[3].angle.getRadians());

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

  public void autoOutputModuleStates(SwerveModuleState[] states) {

    //The maximum voltage value of 10 is taken from Trajectories
    leftModuleFront.setDesiredState(states[0]);
    leftModuleBack.setDesiredState(states[0]);
    rightModuleFront.setDesiredState(states[0]);
    rightModuleBack.setDesiredState(states[0]);

  }

  public void autoOutputSDSModuleStates(SwerveModuleState[] states) {

    //The maximum voltage value of 10 is taken from Trajectories
    leftSDSModuleFront.set(states[0].speedMetersPerSecond / Constants.Drivetrain.kMaxSpeedMetersPerSeconds * 10, states[0].angle.getRadians());
    leftSDSModuleBack.set(states[1].speedMetersPerSecond / Constants.Drivetrain.kMaxSpeedMetersPerSeconds * 10, states[1].angle.getRadians());
    rightSDSModuleFront.set(states[2].speedMetersPerSecond / Constants.Drivetrain.kMaxSpeedMetersPerSeconds * 10, states[2].angle.getRadians());
    rightSDSModuleBack.set(states[2].speedMetersPerSecond / Constants.Drivetrain.kMaxSpeedMetersPerSeconds * 10, states[2].angle.getRadians());

  }

  public void stopDrive() {

    SwerveModuleState[] stopStates = {
      new SwerveModuleState(0, leftModuleFront.getState().angle),
      new SwerveModuleState(0, leftModuleBack.getState().angle),
      new SwerveModuleState(0, rightModuleFront.getState().angle),
      new SwerveModuleState(0, rightModuleBack.getState().angle)};


    autoOutputModuleStates(stopStates);
    
  }

}
