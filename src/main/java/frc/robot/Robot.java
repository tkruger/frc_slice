// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.Subsystems.*;

/**
 * This is a demo program showing the use of the DifferentialDrive class. Runs the motors with
 * arcade steering.
 */
public class Robot extends TimedRobot {

  //Creates Joystick objects
  public static final Joystick leftJoystick = new Joystick(0);
  public static final Joystick rightJoystick = new Joystick(1);

  @Override
  public void robotInit() {

    //Initiates the drivetrain
    Drivetrain.drivetrainInit();
    GyroCode.gyroInit();

  }

  @Override
  public void teleopPeriodic() {

    //Run subsystem code on each tick
    Drivetrain.runDrivetrain();
    Indexer.runIndexer();
    Climber.climbArms();

  }
}
