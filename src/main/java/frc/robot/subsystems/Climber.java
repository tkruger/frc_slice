// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.*;
import com.revrobotics.CANSparkMax;

public class Climber extends SubsystemBase {
  // Instance Variables
  private final CANSparkMax leftClimberMotor, rightClimberMotor;
  private final MotorControllerGroup climberMotors;
  public final RelativeEncoder leftClimberEncoder, rightClimberEncoder;

  // Constructor
  public Climber() {
    // Assign individual motors
    rightClimberMotor = new CANSparkMax(Constants.climber_LEFT_MOTOR_PORT, MotorType.kBrushless);
    leftClimberMotor = new CANSparkMax(Constants.climber_RIGHT_MOTOR_PORT, MotorType.kBrushless);

    // Reset motors
    rightClimberMotor.restoreFactoryDefaults();
    leftClimberMotor.restoreFactoryDefaults();

    // Assign motor controller group
    climberMotors = new MotorControllerGroup(rightClimberMotor, leftClimberMotor);

    // Assign Encoders
    leftClimberEncoder = leftClimberMotor.getEncoder();
    rightClimberEncoder = rightClimberMotor.getEncoder();

  }


  @Override
  public void periodic() {
    // Gets distance travelled
    leftClimberEncoder.getPosition();
    rightClimberEncoder.getPosition();

    // Gets encoder current rate
    leftClimberEncoder.getVelocity();
    rightClimberEncoder.getVelocity();

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void setClimber(double climberSpeed) {
    climberMotors.set(climberSpeed);
  }

  public void setLeftClimber(double climberSpeed) {
    leftClimberMotor.set(climberSpeed);
  }

  public void setRightClimber(double climberSpeed) {
    rightClimberMotor.set(climberSpeed);
  }
}