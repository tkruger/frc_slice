// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class Pneumatics extends SubsystemBase {

  //Variables for subsytem
  private final DoubleSolenoid dSolenoid;
  private final Compressor compressor;


  private boolean compressorEnabled;
  private double compressorPressure;


  /** Creates a new Pnuematics. */
  public Pneumatics() {

    //Initializing Variables
    compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
    dSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);

    compressorEnabled = compressor.enabled();
    compressorPressure = compressor.getPressure();
    
  }

  /* This method sets the Double Solenoid to whichever mode it needs to be */
  public void setSolenoid(DoubleSolenoid.Value pneumaticMode){
    dSolenoid.set(pneumaticMode); //sets pneumatics mode to either forward or reverse
  }

  /* These methods toggle the compressor on and off */
  public void toggleCompressorOn(){
    compressor.enableDigital();
  }
  public void toggleCompressorOff(){
    compressor.disable();
  }

  @Override
  // This method will be called once per scheduler run
  public void periodic() {
    SmartDashboard.putBoolean("Compressor Enabled:", compressorEnabled);
    SmartDashboard.putNumber("Compressor Pressure (PSI)", compressorPressure);
  }
}
