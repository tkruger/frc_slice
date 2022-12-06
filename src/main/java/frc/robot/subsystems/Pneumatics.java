// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pneumatics extends SubsystemBase {

  //Variables for subsytem
  private final DoubleSolenoid dSolenoid;
  private final Compressor compressor;


  /* unused variables 
  private boolean compressorEnabled;
  private double compressorPressure;
  */

  /** Creates a new Pnuematics. */
  public Pneumatics() {

    //Initializing Variables
    compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
    dSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);

    /*Unused variables (for now)
    compressorEnabled = compressor.enabled();
    compressorPressure = compressor.getPressure();
    */
    
  }

  public void setSolenoid(DoubleSolenoid.Value pneumaticMode){
    dSolenoid.set(pneumaticMode); //sets pneumatics mode to either forward or reverse
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
