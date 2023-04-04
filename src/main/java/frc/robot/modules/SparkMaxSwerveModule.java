// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.modules;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.factories.SparkMaxFactory;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;


public class SparkMaxSwerveModule {
  private final CANSparkMax driveMotor;
  private final CANSparkMax steerMotor;

  private final RelativeEncoder driveEncoder;
  private final AbsoluteEncoder steerEncoder;

  private final SparkMaxPIDController drivePIDController;
  private final SparkMaxPIDController steerPIDController;

  private double chassisAngularOffset = 0;
  private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());

  private double driveVelocitySetpoint = 0;
  private double steerAngleSetpoint = 0;

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public SparkMaxSwerveModule(int driveMotorPort, int steerMotorPort, double chassisAngularOffset) {
    driveMotor = SparkMaxFactory.createDefaultSparkMax(driveMotorPort);
    steerMotor = SparkMaxFactory.createDefaultSparkMax(steerMotorPort);

    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    driveMotor.restoreFactoryDefaults();
    steerMotor.restoreFactoryDefaults();

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    driveEncoder = driveMotor.getEncoder();
    steerEncoder = steerMotor.getAbsoluteEncoder(Type.kDutyCycle);

    drivePIDController = driveMotor.getPIDController();
    steerPIDController = steerMotor.getPIDController();

    drivePIDController.setFeedbackDevice(driveEncoder);
    steerPIDController.setFeedbackDevice(steerEncoder);

    // Apply position and velocity conversion factors for the driving encoder. The
    // native units for position and velocity are rotations and RPM, respectively,
    // but we want meters and meters per second to use with WPILib's swerve APIs.
    driveEncoder.setPositionConversionFactor(Constants.Drivetrain.DISTANCE_CONVERSION_FACTOR);
    driveEncoder.setVelocityConversionFactor(Constants.Drivetrain.LINEAR_VELOCITY_CONVERSION_FACTOR);

    // Apply position and velocity conversion factors for the turning encoder. We
    // want these in radians and radians per second to use with WPILib's swerve
    // APIs.
    steerEncoder.setPositionConversionFactor(Constants.Drivetrain.ANGLE_CONVERSION_FACTOR);
    steerEncoder.setVelocityConversionFactor(Constants.Drivetrain.ANGULAR_VELOCITY_CONVERSION_FACTOR);

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    //steerEncoder.setInverted(true);

    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.
    steerPIDController.setPositionPIDWrappingEnabled(true);
    steerPIDController.setPositionPIDWrappingMinInput(0);
    steerPIDController.setPositionPIDWrappingMaxInput(Math.PI * 2);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    driveMotor.burnFlash();
    steerMotor.burnFlash();

    this.chassisAngularOffset = chassisAngularOffset;
    desiredState.angle = new Rotation2d(steerEncoder.getPosition());
    driveEncoder.setPosition(0);
  }

  /**
   * Sets the drive motor of the module to specified proportional, integral, derivative, and feedforward gains.
   * 
   * @param kP The desired proportional gain for the drive motor to be set to.
   * @param kI The desired integral gain for the drive motor to be set to.
   * @param kD The desired derivative gain for the drive motor to be set to.
   * @param kF The desired feedforward gain for the drive motor to be set to.
   */
  public void setDrivePIDF(double kP, double kI, double kD, double kFF) {

    drivePIDController.setP(kP);
    drivePIDController.setI(kI);
    drivePIDController.setD(kD);
    drivePIDController.setFF(kFF);

  }

  /**
   * Sets the steer motor of the module to specified proportional, integral, and derivative gains.
   * 
   * @param kP The desired proportional gain for the steer motor to be set to.
   * @param kI The desired integral gain for the steer motor to be set to.
   * @param kD The desired derivative gain for the steer motor to be set to.
   */
  public void setSteerPID(double kP, double kI, double kD) {

    steerPIDController.setP(kP);
    steerPIDController.setI(kI);
    steerPIDController.setD(kD);

  }

  /**
   * Sets the maximum output range of the native drive PID controller of the module.
   * 
   * @param max The desired maximum forward and reverse power output for the native drive PID controller to be set to.
   */
  public void setMaxDriveOutput(double max) {

    drivePIDController.setOutputRange(-max, max);

  }

  /**
   * Sets the maximum output range of the native steer PID controller of the module.
   * 
   * @param max The desired maximum forward and reverse power output for the native steer PID controller to be set to.
   */
  public void setMaxSteerOutput(double max) {

    steerPIDController.setOutputRange(-max, max);

  }

  /**
   * Sets the idle mode of the drive motor of the module to either brake mode or coast mode.
   * 
   * @param setBrakeMode Whether or not the idle mode of the 
   * drive motor should be set to brake mode(false to set to coast mode).
   */
  public void setDriveIdleMode(boolean setBrakeMode) {

    if(setBrakeMode) {

        driveMotor.setIdleMode(IdleMode.kBrake);

    }
    else {

        driveMotor.setIdleMode(IdleMode.kCoast);

    }

  }
  
 /**
   * Sets the idle mode of the steer motor of the module to either brake mode or coast mode.
   * 
   * @param setBrakeMode Whether or not the idle mode of the 
   * steer motor should be set to brake mode(false to set to coast mode).
   */
  public void setSteerIdleMode(boolean setBrakeMode) {

    if(setBrakeMode) {

        steerMotor.setIdleMode(IdleMode.kBrake);

    }
    else {

        steerMotor.setIdleMode(IdleMode.kCoast);

    }

  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(driveEncoder.getVelocity(),
        new Rotation2d(steerEncoder.getPosition() - chassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        driveEncoder.getPosition(),
        new Rotation2d(steerEncoder.getPosition() - chassisAngularOffset));
  }

  /**
   * Returns the PID target drive velocity of the module(meters/second).
   * 
   * @return The PID target drive velocity of the module(meters/second).
   */
  public double getTargetDriveVelocity() {

    return driveVelocitySetpoint;

  }

  /**
   * Returns the PID target steer angle of the module(radians).
   * 
   * @return The PID target steer angle of the module(radians).
   */
  public double getTargetSteerAngle() {

    return steerAngleSetpoint;

  }

  /**
   * Sets the desired state for the module using native PID controllers for the linear velocity and angle of the module.
   * 
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredStatePID(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
        new Rotation2d(steerEncoder.getPosition()));

    driveVelocitySetpoint = optimizedDesiredState.speedMetersPerSecond;
    steerAngleSetpoint = optimizedDesiredState.angle.getRadians();

    // Command driving and turning SPARKS MAX towards their respective setpoints.
    drivePIDController.setReference(optimizedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
    steerPIDController.setReference(optimizedDesiredState.angle.getRadians(), ControlType.kPosition);

    this.desiredState = desiredState;
  }
  
  /**
   * Sets the desired state for the module using a native PID controller only for the angle of the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
        new Rotation2d(steerEncoder.getPosition()));

    steerAngleSetpoint = optimizedDesiredState.angle.getRadians();

    driveMotor.setVoltage(optimizedDesiredState.speedMetersPerSecond / Constants.Drivetrain.kMaxVelocityMetersPerSecond * 10);
    steerPIDController.setReference(optimizedDesiredState.angle.getRadians(), ControlType.kPosition);

    this.desiredState = desiredState;
  }

  /** Zeroes the SwerveModule drive encoder. */
  public void resetDriveEncoder() {
    driveEncoder.setPosition(0);
  }
}