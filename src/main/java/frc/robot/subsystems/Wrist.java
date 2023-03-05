// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

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
  private final ShuffleboardTab manipulatorTab;
  private final SimpleWidget angleWidget, velocityWidget;

  /** Creates a new Wrist. */
  public Wrist() {
    motor = SparkMaxFactory.createDefaultSparkMax(Constants.Wrist.MOTOR_PORT);
    encoder = motor.getEncoder();
    pidController = motor.getPIDController();

    stowLimitSwitch = new DigitalInput(Constants.Wrist.LIMIT_SWITCH_CHANNEL);

    manipulatorTab = Shuffleboard.getTab("Manipulator Tab");

    angleWidget = manipulatorTab.add("Wrist Angle", 0).withPosition(5, 1).withSize(2, 1);
    velocityWidget = manipulatorTab.add("Wrist Velocity", 0).withPosition(5, 2).withSize(2, 1);
  }

  public void spinWrist(double speed) {
    motor.set(speed);
  }

  public void setWristPosition(double position) {
    pidController.setReference(position, ControlType.kPosition);
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
    boolean stowed = stowLimitSwitch.get();
    return stowed;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (completelyStowed()) {
      setEncoder(Constants.Wrist.MAX_ANGLE);
    }

    angleWidget.getEntry().setDouble(getAngle());
    velocityWidget.getEntry().setDouble(getVelocity());
  }
}
