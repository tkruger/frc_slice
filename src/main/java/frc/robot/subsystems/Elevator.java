// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.factories.SparkMaxFactory;

public class Elevator extends SubsystemBase {
 
  private final CANSparkMax leftMotor, rightMotor;

  private final RelativeEncoder leftEncoder, rightEncoder;

  private final SparkMaxPIDController leftPID, rightPID;

  private final DigitalInput lowLimitSwitch;

  private final ShuffleboardTab teleopTab;

  private final GenericEntry positionWidget, velocityWidget, lowLimitWidget, targetPositionWidget;

  private double targetPosition;

  private boolean calibrating;

  /** Creates a new Elevator. */
  public Elevator() {

    leftMotor = SparkMaxFactory.createDefaultSparkMax(Constants.Elevator.LEFT_PORT);
    rightMotor = SparkMaxFactory.createDefaultSparkMax(Constants.Elevator.RIGHT_PORT);

    leftEncoder = leftMotor.getEncoder(Type.kHallSensor, Constants.Encoder.CPR);
    rightEncoder = rightMotor.getEncoder(Type.kHallSensor, Constants.Encoder.CPR);

    leftPID = leftMotor.getPIDController();
    rightPID = rightMotor.getPIDController();

    lowLimitSwitch = new DigitalInput(Constants.Elevator.LIMIT_SWITCH_CHANNEL);

    teleopTab = Shuffleboard.getTab("Teleop Tab");

    positionWidget = teleopTab.add("Elevator Position", 0).withPosition(2, 0).withSize(2, 1).getEntry();
    velocityWidget = teleopTab.add("Elevator Velocity", 0).withPosition(0, 1).withSize(2, 1).getEntry();
    lowLimitWidget = teleopTab.add("Elevator At Low Limit", false).withPosition(6, 1).withSize(1, 1).getEntry();
    targetPositionWidget = teleopTab.add("Elevator Target Position", 0).getEntry();

    targetPosition = 0;

    calibrating = false;

    setPID(Constants.Elevator.KP, Constants.Elevator.KI, Constants.Elevator.KD);
  }

  /**
   * Runs both elevator motors at the same desired speed(-1, 1).
   * 
   * @param speed The desired speed for both elevator motors to run at(-1, 1).
   */
  public void runElevator(double speed) {

      leftMotor.set(-speed);
      rightMotor.set(speed);

      calibrating = speed <= 0;

  }


  public void setPID(double kP, double kI, double kD) {
    leftPID.setP(kP);
    leftPID.setI(kI);
    leftPID.setD(kD);
    rightPID.setP(kP);
    rightPID.setI(kI);
    rightPID.setD(kD);

    leftPID.setOutputRange(-Constants.Elevator.POSITIONAL_MAX_SPEED, Constants.Elevator.POSITIONAL_MAX_SPEED);
    rightPID.setOutputRange(-Constants.Elevator.POSITIONAL_MAX_SPEED, Constants.Elevator.POSITIONAL_MAX_SPEED);
  }

  /**
   * Uses PID controllers to make the elevator go to a specified position
   * 
   * @param position The position the elevator will go to in rotations
   */
  public void setPosition(double position) {
    leftPID.setReference(position, ControlType.kPosition);
    rightPID.setReference(-position, ControlType.kPosition);
    targetPosition = position;

    calibrating = false;
  }

  /**
   * Calculates and returns the average of the left and right elevator motor positions(rotations).
   * 
   * @return The average of the left and right elevator motor positions(rotations).
   */
  public double getElevatorPosition() {

    return (getLeftMotorPosition() + -getRightMotorPosition()) / 2;

  }


  /**
   * Obtains and returns the current position of the left elevator motor(rotations).
   * 
   * @return The current position of the left elevator motor(rotations).
   */
  public double getLeftMotorPosition() {

    return leftEncoder.getPosition();

  }

  /**
   * Obtains and returns the current position of the right elevator motor(rotations).
   * 
   * @return The current position of the right elevator motor(rotations).
   */
  public double getRightMotorPosition() {

    return rightEncoder.getPosition();

  }

  /**
   * Resets the encoder to a desired positon (rotations)
   * 
   * @param position The desired position to set the motors to(rotations).
   */
  public void setEncoderPosition(double position) {

    leftEncoder.setPosition(position);
    rightEncoder.setPosition(position);

  }

  /**
   * Calculates and returns the average of the left and right elevator motor velocities(rpm).
   * 
   * @return The average of the left and right elevator motor velocities.(rpm)
   */
  public double getElevatorVelocity() {

    return (getLeftMotorVelocity() + -getRightMotorVelocity()) / 2;

  }

  /**
   * Obtains and returns the current velocity of the left elevator motor(rpm).
   * 
   * @return The current velocity of the left elevator motor(rpm).
   */
  public double getLeftMotorVelocity() {

    return leftEncoder.getVelocity();

  }

   /**
   * Obtains and returns the current velocity of the right elevator motor(rpm).
   * 
   * @return The current velocity of the right elevator motor(rpm).
   */
  public double getRightMotorVelocity() {

    return rightEncoder.getVelocity();

  }

  /**
   * 
   * @return the average output velocity of the two motors, in amps
   */
  public double getAverageOutputCurrent() {
    return (leftMotor.getOutputCurrent() + rightMotor.getOutputCurrent()) / 2;
  }

  public boolean atBottom() {
    return lowLimitSwitch.get();
  }

  public boolean atTargetPosition() {
    double position = getElevatorPosition();
    double error = Math.abs(position - targetPosition);
    return error < Constants.Elevator.POSITIONAL_ERROR_THRESHOLD;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if(calibrating) {
      if(atBottom()) {
        setEncoderPosition(0);
      }
    }

    positionWidget.setDouble(getElevatorPosition());
    velocityWidget.setDouble(getElevatorVelocity());
    lowLimitWidget.setBoolean(atBottom());
    targetPositionWidget.setDouble(targetPosition);
  }

}
