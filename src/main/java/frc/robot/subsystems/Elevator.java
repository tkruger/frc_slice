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

public class Elevator extends SubsystemBase {
 
  private final CANSparkMax leftMotor;
  private final CANSparkMax rightMotor;

  private final RelativeEncoder leftEncoder;
  private final RelativeEncoder rightEncoder;

  /** Creates a new Elevator. */
  public Elevator() {

    leftMotor = new CANSparkMax(Constants.elevator_LEFT_PORT, MotorType.kBrushless);
    rightMotor = new CANSparkMax(Constants.elevator_RIGHT_PORT, MotorType.kBrushless);

    leftMotor.restoreFactoryDefaults();
    rightMotor.restoreFactoryDefaults();

    leftMotor.setIdleMode(IdleMode.kBrake);
    rightMotor.setIdleMode(IdleMode.kBrake);

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

    if(runUpwards == true) {

      leftMotor.set(speed);

    }
    else {

      leftMotor.set(-speed);

    }

  }

  public void runRightMotor(boolean runUpwards, double speed) {

    if(runUpwards == true) {

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
