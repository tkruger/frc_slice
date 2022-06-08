// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

public class Indexer extends SubsystemBase {
  /** Creates a new Indexer. */

  CANSparkMax indexerMotor;

  public Indexer() {

    final CANSparkMax indexerMotor = new CANSparkMax(Constants.INDEXER_INDEXER_MOTOR, CANSparkMaxLowLevel.MotorType .kBrushless);

  }

  public void runIndexer(boolean indexerMotorForward, boolean indexerMotorBackward) {

    indexerMotor.setIdleMode(CANSparkMax.IdleMode .kCoast);

    if(indexerMotorBackward == false) {
      if(indexerMotorForward == true) {
        indexerMotor.set(1);
      }
    
      if(indexerMotorForward ==false) {
      indexerMotor.set(0);
      }
    }
    if(indexerMotorForward == false) {
      if(indexerMotorBackward == true) {
        indexerMotor.set(-1);
      }

      if(indexerMotorBackward == false) {
        indexerMotor.set(0);
      }
      
    }

    indexerMotor.setControlFramePeriodMs(50);

    System.out.println(indexerMotor.get());
    System.out.println(indexerMotor.getOutputCurrent());
    System.out.println(indexerMotor.getMotorTemperature());
    System.out.println(indexerMotor.getLastError());
    System.out.println(indexerMotor.getFaults());

  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
