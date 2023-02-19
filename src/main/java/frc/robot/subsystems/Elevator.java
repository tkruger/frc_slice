// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.factories.SparkMaxFactory;

public class Elevator extends SubsystemBase {
 
  private final CANSparkMax leftMotor, rightMotor;

  private final RelativeEncoder leftEncoder, rightEncoder;

  private final SparkMaxPIDController leftPID, rightPID;

  /** Creates a new Elevator. */
  public Elevator() {

    leftMotor = SparkMaxFactory.createDefaultSparkMax(Constants.elevator_LEFT_PORT);
    rightMotor = SparkMaxFactory.createDefaultSparkMax(Constants.elevator_RIGHT_PORT);

    leftEncoder = leftMotor.getEncoder(Type.kHallSensor, Constants.ENCODER_CPR);
    rightEncoder = rightMotor.getEncoder(Type.kHallSensor, Constants.ENCODER_CPR);

    leftPID = leftMotor.getPIDController();
    rightPID = rightMotor.getPIDController();

  }

  /**
   * Runs both elevator motors at the same desired speed and direction.
   * 
   * @param runUpwards Whether both elevator motors should run upwards or not.
   * @param speed The desired speed for both elevators motors to run at.
   */
  public void runElevator(boolean runUpwards, double speed) {

    if(runUpwards == true) {

      leftMotor.set(speed);
      rightMotor.set(speed);

    }
    else {

      leftMotor.set(-speed);
      rightMotor.set(-speed);

    }

  }

  /**
   * Runs only the left elevator motor at a desired speed and direction.
   * 
   * @param runUpwards Whether the left elevator motor should run upwards or not.
   * @param speed The desired speed for the left elevator motor to run at.
   */
  public void runLeftMotor(boolean runUpwards, double speed) {

    if(runUpwards) {

      leftMotor.set(speed);

    }
    else {

      leftMotor.set(-speed);

    }

  }

   /**
   * Runs only the right elevator motor at a desired speed and direction.
   * 
   * @param runUpwards Whether the right elevator motor should run upwards or not.
   * @param speed The desired speed for the right elevator motor to run at.
   */
  public void runRightMotor(boolean runUpwards, double speed) {

    if(runUpwards) {

      rightMotor.set(speed);

    }
    else {

      rightMotor.set(-speed);

    }

  }

  public void setPID(double kP, double kI, double kD) {
    leftPID.setP(kP);
    leftPID.setI(kI);
    leftPID.setD(kD);
    rightPID.setP(kP);
    rightPID.setI(kI);
    rightPID.setD(kD);
  }

  public void setPosition(double position) {
    leftPID.setReference(position, ControlType.kPosition);
    rightPID.setReference(position, ControlType.kPosition);
  }

  public void setEncoders(double position) {
    leftEncoder.setPosition(position);
    rightEncoder.setPosition(position);
  }

  /**
   * Obtains and returns the current position of the left elevator encoder.
   * 
   * @return The current position of the left elevator encoder.
   */
  public double getLeftEncoderPosition() {

    return leftEncoder.getPosition();

  }

  /**
   * Obtains and returns the current position of the right elevator encoder.
   * 
   * @return The current position of the right elevator encoder.
   */
  public double getRightEncoderPosition() {

    return rightEncoder.getPosition();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
