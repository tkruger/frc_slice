// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.factories.SparkMaxFactory;

public class Elevator extends SubsystemBase {
 
  private final CANSparkMax leftMotor, rightMotor;

  private final RelativeEncoder leftEncoder, rightEncoder;

  private final SparkMaxPIDController leftPID, rightPID;

  /** Creates a new Elevator. */
  public Elevator() {

    leftMotor = SparkMaxFactory.createDefaultSparkMax(Constants.Elevator.LEFT_PORT);
    rightMotor = SparkMaxFactory.createDefaultSparkMax(Constants.Elevator.RIGHT_PORT);

    leftEncoder = leftMotor.getEncoder(Type.kHallSensor, Constants.Encoder.CPR);
    rightEncoder = rightMotor.getEncoder(Type.kHallSensor, Constants.Encoder.CPR);

    leftPID = leftMotor.getPIDController();
    rightPID = rightMotor.getPIDController();

  }

  /**
   * Runs both elevator motors at the same desired speed(-1, 1).
   * 
   * @param speed The desired speed for both elevator motors to run at(-1, 1).
   */
  public void runElevator(double speed) {

      leftMotor.set(-speed);
      rightMotor.set(speed);

  }

   /**
   * Runs the left elevator motor at a desired speed(-1, 1).
   * 
   * @param speed The desired speed for the left elevator motor to run at(-1, 1).
   */
  public void runLeftMotor(double speed) {

    leftMotor.set(speed);

  } 

  /**
   * Runs the right elevator motor at a desired speed(-1, 1).
   * 
   * @param speed The desired speed for the right elevator motor to run at(-1, 1).
   */
  public void runRightMotor(double speed) {

    rightMotor.set(speed);

  }


  public void setPID(double kP, double kI, double kD) {
    leftPID.setP(kP);
    leftPID.setI(kI);
    leftPID.setD(kD);
    rightPID.setP(kP);
    rightPID.setI(kI);
    rightPID.setD(kD);
  }

  /**
   * Uses PID controllers to make the elevator go to a specified position
   * 
   * @param position The position the elevator will go to in rotations
   */
  public void setPosition(double position) {
    leftPID.setReference(position, ControlType.kPosition);
    rightPID.setReference(-position, ControlType.kPosition);
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
    return getAverageOutputCurrent() > 50;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Left Elevator Motor Position", getLeftMotorPosition());
    SmartDashboard.putNumber("Right Elevator Motor Position", getRightMotorPosition());
    SmartDashboard.putNumber("Elevator Motor Current", (leftMotor.getOutputCurrent() + rightMotor.getOutputCurrent()) / 2);
  }

}