// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.factories.SparkMaxFactory;

public class Intake extends SubsystemBase {
 
  private final CANSparkMax pivotMotor, rotateMotor;

  private final RelativeEncoder pivotEncoder, rotateEncoder;

  /** Creates a new Elevator. */
  public Intake() {

    pivotMotor = SparkMaxFactory.createDefaultSparkMax(Constants.intake_PIVOT_PORT);
    rotateMotor = SparkMaxFactory.createDefaultSparkMax(Constants.intake_ROTATE_PORT);

    pivotEncoder = pivotMotor.getEncoder(Type.kHallSensor, Constants.ENCODER_CPR);
    rotateEncoder = rotateMotor.getEncoder(Type.kHallSensor, Constants.ENCODER_CPR);

  }

  public void runModeSwitcher(boolean close, double speed) {

    if(close) {

      pivotMotor.set(speed);

    }
    else {

      pivotMotor.set(-speed);

    }

  }

  public void runIntake(boolean eject, double speed) {

    if(eject) {

      rotateMotor.set(speed);

    }
    else {

      rotateMotor.set(-speed);

    }

  }

  public double getLeftEncoderPosition() {

    return pivotEncoder.getPosition();

  }

  public double getRightEncoderPosition() {

    return rotateEncoder.getPosition();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
