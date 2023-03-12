// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.factories.SparkMaxFactory;


/**
 * Controls the wrist that angles the intake up and down
 */
public class Wrist extends SubsystemBase {

  private final CANSparkMax motor;
  private final RelativeEncoder encoder;
  private final SparkMaxPIDController pidController;
  private final DigitalInput stowLimitSwitch;
  private final ShuffleboardTab teleopTab;
  private final GenericEntry angleWidget, velocityWidget;//, voltageWidget;

  private double targetPosition;

  /** Creates a new Wrist. */
  public Wrist() {
    motor = SparkMaxFactory.createDefaultSparkMax(Constants.Wrist.MOTOR_PORT);
    encoder = motor.getEncoder();
    pidController = motor.getPIDController();

    encoder.setPositionConversionFactor(Constants.Wrist.POSITION_CONVERSION_FACTOR);

    stowLimitSwitch = new DigitalInput(Constants.Wrist.LIMIT_SWITCH_CHANNEL);

    teleopTab = Shuffleboard.getTab("Teleop Tab");

    angleWidget = teleopTab.add("Wrist Angle", 0).withPosition(5, 0).withSize(2, 1).getEntry();
    velocityWidget = teleopTab.add("Wrist Velocity", 0).withPosition(7, 1).withSize(2, 1).getEntry();
  
    targetPosition = -105;
  }

  public void spinWrist(double speed) {
    motor.set(speed);
  }

  public void setWristPosition(double position) {
    pidController.setReference(position, ControlType.kPosition);
    targetPosition = position;
  }

  /**
   * Sets the gains of the positional PID controller for the wrist motor
   * @param kP The proportional gain of the PID controller
   * @param kI The integral gain of the PID controller
   * @param kD The derivative gain of the PID controller
   */
  public void setPID(double kP, double kI, double kD) {
    pidController.setP(kP);
    pidController.setI(kI);
    pidController.setD(kD);

    pidController.setIAccum(0);
    pidController.setOutputRange(-Constants.Wrist.POSITIONAL_MAX_SPEED, Constants.Wrist.POSITIONAL_MAX_SPEED);
  }

  public void setPID(double kP, double kI, double kD, double maxSpeed) {
    pidController.setP(kP);
    pidController.setI(kI);
    pidController.setD(kD);

    pidController.setIAccum(0);
    pidController.setOutputRange(-maxSpeed, maxSpeed);
  }

  public void setPositionalMaxSpeed(double maxSpeed) {
    pidController.setOutputRange(-maxSpeed, maxSpeed);
  }
  /**
   * Resets the encoder position to a set angle
   * @param angleDegrees the current angle fo the wrist, in degrees
   */
  public void setEncoder(double angleDegrees) {
    encoder.setPosition(angleDegrees);
  }

  /**
   * @return the current angle of the wrist in degrees from the horizontal. Positive is above horizontal, negative is below
   */
  public double getAngle() {
    double angle = encoder.getPosition();
    return angle;
  }

  /**
   * @return the angular velocity of the wrist in RPM
   */
  public double getVelocity() {
    double velocity = encoder.getVelocity();
    return velocity;
  }

  /**
   * @return true if the intake is completely stowed, false otherwise
   */
  public boolean completelyStowed() {
    boolean stowed = motor.getOutputCurrent() > Constants.Wrist.CALIBRATE_CURRENT_THRESHOLD;
    return stowed;
  }

  public boolean atTargetPosition() {
    double position = getAngle();
    double error = Math.abs(position - targetPosition);
    return error < Constants.Wrist.POSITIONAL_ERROR_THRESHOLD;

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    angleWidget.setDouble(getAngle());
    velocityWidget.setDouble(getVelocity());
    //voltageWidget.setDouble(motor.getOutputCurrent());
  }
}
