// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;

/**
 * This is a demo program showing the use of the DifferentialDrive class. Runs the motors with
 * arcade steering.
 */
public class Robot extends TimedRobot {

  //Creates Joystick objects
  public static final Joystick leftJoystick = new Joystick(0);
  public static final Joystick rightJoystick = new Joystick(1);

  // Creates an ADXRS450_Gyro object on the onboard SPI port
  ADXRS450_Gyro gyro = new ADXRS450_Gyro();

  @Override
  public void robotInit() {

    //Initiates the drivetrain
    Drivetrain.drivetrainInit();

    //Calibrates the Gyro before movement
    gyro.calibrate();

  }

  @Override
  public void teleopPeriodic() {

    //Run subsystem code on each tick
    Drivetrain.runDrivetrain();
    Indexer.runIndexer();
    Climber.climbArms();

  }
}
