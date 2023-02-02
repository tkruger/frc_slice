// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.drivers.SparkMaxFactory;

public class Elevator extends SubsystemBase {
 
  private final CANSparkMax leftMotor, rightMotor;

  private final RelativeEncoder leftEncoder, rightEncoder;

  /** Creates a new Elevator. */
  public Elevator() {

    leftMotor = SparkMaxFactory.createDefaultSparkMax(Constants.elevator_LEFT_PORT);
    rightMotor = SparkMaxFactory.createDefaultSparkMax(Constants.elevator_RIGHT_PORT);

    leftEncoder = leftMotor.getEncoder(Type.kHallSensor, Constants.ENCODER_CPR);
    rightEncoder = rightMotor.getEncoder(Type.kHallSensor, Constants.ENCODER_CPR);

  }

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

  public void runLeftMotor(boolean runUpwards, double speed) {

    if(runUpwards) {

      leftMotor.set(speed);

    }
    else {

      leftMotor.set(-speed);

    }

  }

  public void runRightMotor(boolean runUpwards, double speed) {

    if(runUpwards) {

      rightMotor.set(speed);

    }
    else {

      rightMotor.set(-speed);

    }

  }

  public double getLeftEncoderPosition() {

    return leftEncoder.getPosition();

  }

  public double getRightEncoderPosition() {

    return rightEncoder.getPosition();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
