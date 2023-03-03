// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pneumatics extends SubsystemBase {

  //Declaring Variables
  private final DoubleSolenoid dSolenoid;

  private final Compressor compressor;
  private final Boolean compressor_enabled;
  private double compressor_pressure;
  private double compressor_current;


  /** Creates a new Pneumatics. */
  public Pneumatics() {

    //Initializing Variables
    dSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);

    compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
    compressor_enabled = compressor.isEnabled();
    compressor_pressure = compressor.getPressure();
    compressor_current = compressor.getCurrent();
  }

  //This method sets the Double Solenoid to either forward, reverse, or off
  public void setSolenoid(DoubleSolenoid.Value pnuematicMode){
    dSolenoid.set(pnuematicMode); //sets the Solenoid to the mode that is passed through
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
