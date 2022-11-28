// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/* The yaw and pitch motor will rotate the cannon a full 360 degrees
 * and will lift it from a 15 degree angle to a 75 degree angle */

public class Motors extends SubsystemBase {
  //Declaring the motor variables
  TalonFX yawMotor;
  TalonFXConfiguration yawMotor_Config;
  CANSparkMax pitchMotor;

  /** Creates a new Motors Subsystem (for the yaw and pitch motor). */
  public Motors() {
    //Initiating Motors
    yawMotor = new TalonFX(0);
    yawMotor_Config = new TalonFXConfiguration();

    pitchMotor = new CANSparkMax(1, MotorType.kBrushless);
  }

  //This command will run both the yaw and pitch motors
  public void runMotors(double yawMotorPercentOut, double pitchMotorOutVolts){
    //Programming the Yaw Motor
    yawMotor.configAllSettings(yawMotor_Config); //configuring the yaw motor

    yawMotor.set(TalonFXControlMode.PercentOutput, yawMotorPercentOut); //Turns the yaw motor 360 degrees

    //Programming the Pitch Motor
    pitchMotor.setVoltage(pitchMotorOutVolts); //Need to determine the voltage needed to lift the cannon from 15 degree angle to 75 degree angle

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
