// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.factories.SparkMaxFactory;

/**
 * Controls the wrist that angles the intake up and down
 */
public class Wrist extends SubsystemBase {

  // Spark Max Motor Objects
  private final CANSparkMax motor;
  private final RelativeEncoder encoder;
  private final PIDController customPIDController;
  private final SparkMaxPIDController nativePIDController;

  // Shuffleboard
  private final ShuffleboardTab teleopTab;
  private final GenericEntry angleWidget, velocityWidget, targetAngleWidget;//, voltageWidget;

  private double targetPosition;
  private boolean manualControl;

  /** Creates a new Wrist. */
  public Wrist() {
    // Motor
    motor = SparkMaxFactory.createDefaultSparkMax(Constants.Wrist.MOTOR_PORT);

    // Encoder
    encoder = motor.getEncoder();
    encoder.setPositionConversionFactor(Constants.Wrist.POSITION_CONVERSION_FACTOR);
    targetPosition = -105;
    
    // PID
    customPIDController = new PIDController(Constants.Wrist.CUSTOM_KP, Constants.Wrist.CUSTOM_KI, Constants.Wrist.CUSTOM_KD);

    nativePIDController = motor.getPIDController();
    nativePIDController.setP(Constants.Wrist.NATIVE_KP);
    nativePIDController.setI(Constants.Wrist.NATIVE_KI);
    nativePIDController.setD(Constants.Wrist.NATIVE_KD);

    // Shuffleboard
    teleopTab = Shuffleboard.getTab("Teleop Tab");

    angleWidget = teleopTab.add("Wrist Angle", 0).withPosition(5, 0).withSize(2, 1).getEntry();
    velocityWidget = teleopTab.add("Wrist Velocity", 0).withPosition(7, 1).withSize(2, 1).getEntry();
    targetAngleWidget = teleopTab.add("Target Wrist Angle", 0).getEntry();

    manualControl = true;

  }

  public void spinWrist(double speed) {
    manualControl = true;
    motor.set(speed);
  }

  public void setWristVoltage(double voltage) {
    manualControl = true;
    motor.setVoltage(voltage);
  }

  public void setWristPosition(double position) {
    //customPIDController.setSetpoint(position);
    nativePIDController.setReference(position, ControlType.kPosition);
    manualControl = false;
    targetPosition = position;
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

  public void enableManualControl() {
    manualControl = true;
  }

  public void disableManualControl() {
    manualControl = false;
    //customPIDController.reset();
    nativePIDController.setIAccum(0);
  }

  @Override
  public void periodic() {
    updateShuffleboard();
    //updatePIDController();
  }

  public void updateShuffleboard() {
      angleWidget.setDouble(getAngle());
  }

  public void updatePIDController() {
    // Get angle from encoder reading
    if (!manualControl) {
      double angle = getAngle();
      //double feedback = customPIDController.calculate(angle);
      double feedforward = getAntigravityFeedforward(angle);
      //double volts = MathUtil.clamp(feedback + feedforward, -12, 12);
      //motor.setVoltage(volts);
      nativePIDController.setReference(targetPosition, ControlType.kPosition, 1, feedforward);
    }
  }

  /**
   * Calculates the required extra output to keep the wrist stable at the current angle
   * @return the needed feedforward in volts
   */
  public double getAntigravityFeedforward(double angle) {
    if (angle < -80) {
      return 0;
    }
    return (angle * -0.00742061) - 0.670058;
  }
}
