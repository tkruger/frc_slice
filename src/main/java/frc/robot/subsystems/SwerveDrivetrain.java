/// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.*;
import frc.robot.auto.Paths;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

import com.kauailabs.navx.frc.AHRS;

import com.revrobotics.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.swervedrivespecialties.swervelib.*;
import com.swervedrivespecialties.swervelib.SwerveModule;

public class SwerveDrivetrain extends SubsystemBase {
  
  //Creates drivetrain motor objects and groups
  private final CANSparkMax leftMotorFrontDrive, leftMotorFrontSteer, leftMotorBackDrive, leftMotorBackSteer, rightMotorFrontDrive, rightMotorFrontSteer, rightMotorBackDrive, rightMotorBackSteer;
  private final RelativeEncoder leftEncoderFrontDrive, leftEncoderFrontSteer, leftEncoderBackDrive, leftEncoderBackSteer, rightEncoderFrontDrive, rightEncoderFrontSteer, rightEncoderBackDrive, rightEncoderBackSteer;
  private final SwerveModule leftModuleFront, leftModuleBack, rightModuleFront, rightModuleBack;

  private final SwerveDrivePoseEstimator m_poseEstimator;

  private final AHRS navXGyro;

  private static final Field2d field2d = new Field2d();

  public static final SwerveDriveKinematics m_swerveKinematics = new SwerveDriveKinematics(
    // Front left
    new Translation2d(Constants.kTrackWidthMeters / 2.0, 0.8128 / 2.0),
    // Back left
    new Translation2d(Constants.kTrackWidthMeters / 2.0, 0.8128 / 2.0),
    // Front right
    new Translation2d(Constants.kTrackWidthMeters  / 2.0, -0.8128 / 2.0),
    // Back right
    new Translation2d(Constants.kTrackWidthMeters  / 2.0, -0.8128 / 2.0));

  public double leftFrontLastPosition, leftBackLastPosition, rightFrontLastPosition, rightBackLastPosition;
  // The current target position of every motor
  public double leftTargetPositionFront, leftTargetPositionBack, rightTargetPositionFront, rightTargetPositionBack;

  /** Creates a new Drivetrain. */
  public SwerveDrivetrain() {

    leftMotorFrontDrive = new CANSparkMax(Constants.drivetrain_LEFT_FRONT_PORT_DRIVE, MotorType.kBrushless);
    leftMotorFrontSteer = new CANSparkMax(Constants.drivetrain_LEFT_FRONT_PORT_STEER, MotorType.kBrushless);

    leftMotorBackDrive = new CANSparkMax(Constants.drivetrain_LEFT_BACK_PORT_DRIVE, MotorType.kBrushless);
    leftMotorBackSteer = new CANSparkMax(Constants.drivetrain_LEFT_BACK_PORT_STEER, MotorType.kBrushless);

    rightMotorFrontDrive = new CANSparkMax(Constants.drivetrain_RIGHT_FRONT_PORT_DRIVE, MotorType.kBrushless);
    rightMotorFrontSteer = new CANSparkMax(Constants.drivetrain_RIGHT_FRONT_PORT_STEER, MotorType.kBrushless);

    rightMotorBackDrive = new CANSparkMax(Constants.drivetrain_RIGHT_BACK_PORT_DRIVE, MotorType.kBrushless);
    rightMotorBackSteer = new CANSparkMax(Constants.drivetrain_RIGHT_BACK_PORT_STEER, MotorType.kBrushless);

    leftEncoderFrontDrive = createEncoder(leftMotorFrontDrive);
    leftEncoderFrontSteer = createEncoder(leftMotorFrontSteer);

    leftEncoderBackDrive = createEncoder(leftMotorBackDrive);
    leftEncoderBackSteer = createEncoder(leftMotorBackSteer);

    rightEncoderFrontDrive = createEncoder(rightMotorFrontDrive);
    rightEncoderFrontSteer = createEncoder(rightMotorFrontSteer);

    rightEncoderBackDrive = createEncoder(rightMotorBackDrive);
    rightEncoderBackSteer = createEncoder(rightMotorBackSteer);


    //The gear ratios, motor ports, and steer offsets for these object declarations are placholders for now
    leftModuleFront = Mk4iSwerveModuleHelper.createNeo(
      Shuffleboard.getTab("SmartDashboard").getLayout("Left Front Module", BuiltInLayouts.kList), 
      Mk4iSwerveModuleHelper.GearRatio.L1, 
      Constants.drivetrain_LEFT_FRONT_PORT_DRIVE, 
      Constants.drivetrain_LEFT_FRONT_PORT_STEER, 
      Constants.drivetrain_LEFT_FRONT_PORT_STEER, 
      0);

    leftModuleBack = Mk4iSwerveModuleHelper.createNeo(
      Shuffleboard.getTab("SmartDashboard").getLayout("Left Back Module", BuiltInLayouts.kList), 
      Mk4iSwerveModuleHelper.GearRatio.L1, 
      Constants.drivetrain_LEFT_BACK_PORT_DRIVE, 
      Constants.drivetrain_LEFT_BACK_PORT_STEER, 
      Constants.drivetrain_LEFT_BACK_PORT_STEER, 
      0);

    rightModuleFront = Mk4iSwerveModuleHelper.createNeo(
      Shuffleboard.getTab("SmartDashboard").getLayout("Right Front Module", BuiltInLayouts.kList), 
      Mk4iSwerveModuleHelper.GearRatio.L1, 
      Constants.drivetrain_RIGHT_FRONT_PORT_DRIVE, 
      Constants.drivetrain_RIGHT_FRONT_PORT_STEER, 
      Constants.drivetrain_RIGHT_FRONT_PORT_STEER, 
      0);

    rightModuleBack = Mk4iSwerveModuleHelper.createNeo(
      Shuffleboard.getTab("SmartDashboard").getLayout("Right Back Module", BuiltInLayouts.kList), 
      Mk4iSwerveModuleHelper.GearRatio.L1, 
      Constants.drivetrain_RIGHT_BACK_PORT_DRIVE, 
      Constants.drivetrain_RIGHT_BACK_PORT_STEER, 
      Constants.drivetrain_RIGHT_BACK_PORT_STEER, 
      0);

    navXGyro = new AHRS(SerialPort.Port.kUSB1);

    // Display current gyro heading on Shuffleboard
    Shuffleboard.getTab("SmartDashboard").add(navXGyro);    

    //These standard deviation values should be measured proplerly for our robot
    m_poseEstimator = new SwerveDrivePoseEstimator(
      new Rotation2d(Units.degreesToRadians(getHeading())),
      new Pose2d(),
      m_swerveKinematics,
      new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.01), // State measurement standard deviations. X, Y, theta.
      new MatBuilder<>(Nat.N1(), Nat.N1()).fill(0.01), // Local measurement standard deviations. Gyro(theta).
      new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.01)); // Global measurement standard deviations. X, Y, and theta.

  }

  @Override
  public void periodic() {
    
    updateOdometry();

    // This method will be called once per scheduler run
    field2d.setRobotPose(getEstimatedPosition());

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public RelativeEncoder createEncoder(CANSparkMax motor) {

    RelativeEncoder encoder = motor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, Constants.drivetrain_ENCODER_CPR);

    encoder.setVelocityConversionFactor(Constants.drivetrain_VELOCITY_CONVERSION_FACTOR);
    encoder.setPositionConversionFactor(Constants.drivetrain_POSITION_CONVERSION_RATIO);

    return encoder;

  }

  public static void updateField2d(int trajectoryNumber) {

    // Creates and pushes Field2d to SmartDashboard.
    SmartDashboard.putData(field2d);

    // Pushes the trajectory to Field2d.
    try {
      field2d.getObject("Trajectory").setTrajectory(Paths.getAutoPath().get(trajectoryNumber - 1));
    } catch (Exception exception) {
      field2d.getObject("Trajectory").setTrajectory(Paths.returnPlaceholderTrajectory());
    }

  }

  public Pose2d updateOdometry() {

    m_poseEstimator.update(
      new Rotation2d(Units.degreesToRadians(getHeading())), 
      new SwerveModuleState(leftEncoderFrontDrive.getVelocity(), new Rotation2d(Units.rotationsToRadians(leftEncoderFrontSteer.getPosition()))),
      new SwerveModuleState(leftEncoderBackDrive.getVelocity(), new Rotation2d(Units.rotationsToRadians(leftEncoderBackSteer.getPosition()))),
      new SwerveModuleState(rightEncoderFrontDrive.getVelocity(), new Rotation2d(Units.rotationsToRadians(rightEncoderFrontSteer.getPosition()))),
      new SwerveModuleState(rightEncoderBackDrive.getVelocity(), new Rotation2d(Units.rotationsToRadians(rightEncoderBackSteer.getPosition()))));

    //This latency value(0.3) is a place holder for now and should be measured properly for our robot
    // m_poseEstimator.addVisionMeasurement(
    //   getEstimatedGlobalPose(m_poseEstimator.getEstimatedPosition()),
    //   Timer.getFPGATimestamp() - 0.3);

    return m_poseEstimator.getEstimatedPosition();

  }

  public Pose2d getEstimatedPosition() {
    return m_poseEstimator.getEstimatedPosition();
  }

  public void resetOdometry(Pose2d position) {

    leftEncoderFrontDrive.setPosition(0);
    leftEncoderFrontSteer.setPosition(0);
    leftEncoderBackDrive.setPosition(0);
    leftEncoderBackSteer.setPosition(0);
    rightEncoderFrontDrive.setPosition(0);
    rightEncoderFrontSteer.setPosition(0);
    rightEncoderBackDrive.setPosition(0);
    rightEncoderBackSteer.setPosition(0);

    m_poseEstimator.resetPosition(position, new Rotation2d(Units.degreesToRadians(getHeading())));

    //navXGyro.reset();
    //navXGyro.zeroYaw();

  }

  public void swerveDrive(ChassisSpeeds chassisSpeeds) {

    SwerveModuleState[] states = m_swerveKinematics.toSwerveModuleStates(chassisSpeeds);
    //SwerveDriveKinematics.normalizeWheelSpeeds(states, Constants.kMaxSpeedMetersPerSeconds);

    //The maximum voltage value of 10 is taken from Trajectories
    leftModuleFront.set(states[0].speedMetersPerSecond / Constants.kMaxSpeedMetersPerSeconds * 10, states[0].angle.getRadians());
    leftModuleBack.set(states[1].speedMetersPerSecond / Constants.kMaxSpeedMetersPerSeconds * 10, states[1].angle.getRadians());
    rightModuleFront.set(states[2].speedMetersPerSecond / Constants.kMaxSpeedMetersPerSeconds * 10, states[2].angle.getRadians());
    rightModuleBack.set(states[2].speedMetersPerSecond / Constants.kMaxSpeedMetersPerSeconds * 10, states[2].angle.getRadians());

  }

  /*public static double deadband(double value, double deadband) {

    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } 
      else {
        return (value + deadband) / (1.0 - deadband);
      }
    } 
    else {
      return 0.0;
    }
    
  }

  public double modifyAxis(double value) {

    // Deadband
    value = deadband(value, 0.05);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;

  }*/
  
  public double getHeading() {

    return -navXGyro.getYaw() + 180;

  }

  public double getTurnRate() {

    return navXGyro.getRate();

  }

  public void autoOutputModuleStates(SwerveModuleState[] states) {

    //The maximum voltage value of 10 is taken from Trajectories
    leftModuleFront.set(states[0].speedMetersPerSecond / Constants.kMaxSpeedMetersPerSeconds * 10, states[0].angle.getRadians());
    leftModuleBack.set(states[1].speedMetersPerSecond / Constants.kMaxSpeedMetersPerSeconds * 10, states[1].angle.getRadians());
    rightModuleFront.set(states[2].speedMetersPerSecond / Constants.kMaxSpeedMetersPerSeconds * 10, states[2].angle.getRadians());
    rightModuleBack.set(states[2].speedMetersPerSecond / Constants.kMaxSpeedMetersPerSeconds * 10, states[2].angle.getRadians());

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
