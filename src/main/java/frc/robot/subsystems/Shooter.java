// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.*;

public class Shooter extends SubsystemBase {
  
  //Creates shooter motor objects
  private final CANSparkMax primaryFlywheel, secondaryFlywheel;

  /** Creates a new Shooter. */
  public Shooter() {
    
    //Instantiates motors and motor groups
    primaryFlywheel = new CANSparkMax(Constants.intake_FLYWHEEL_PRIMARY_PORT, MotorType.kBrushless);
    secondaryFlywheel = new CANSparkMax(Constants.intake_FLYWHEEL_SECONDARY_PORT, MotorType.kBrushless);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void SetShooters(double primarySpeed, double secondarySpeed) { 

    primaryFlywheel.set(primarySpeed);
    secondaryFlywheel.set(secondarySpeed);
    
  }
}
