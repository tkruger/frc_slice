// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Example extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  // Create variables and objects here

  public Example() {
    // This Constructor will be run when the subsystem is first created in RobotContainer.
    // Instantiate any objects here
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void set(double speed) {
    // This method can be called by commands to set the speed of motors in this subsystem.
    // This method is custom, and can be easily replaced if needed
  }
}