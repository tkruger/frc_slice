// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

import frc.robot.commands.*;
import frc.robot.commands.Wrist.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final Wrist m_wrist = new Wrist();
  //public final ColorSensor m_colorSensor = new ColorSensor();
  public final Joystick leftJoystick = Button.leftJoystick;
  public final Joystick rightJoystick = Button.rightJoystick;
  public final Joystick manipulatorJoystick = Button.manipulatorJoystick;

  public final WristRunCommand m_wristRunUpwards = new WristRunCommand(m_wrist, true);
  public final WristRunCommand m_wristRunDownwards = new WristRunCommand(m_wrist, false);
  public final WristStationaryCommand m_wristStationary = new WristStationaryCommand(m_wrist);
  public final ResetAngleCommand m_resetWristAngle = new ResetAngleCommand(m_wrist);
  public final SetWristPosition m_testSetWristPosition = new SetWristPosition(m_wrist, -50);

  public final ManualVoltageWristCommand m_regressionTester = new ManualVoltageWristCommand(m_wrist, -0.25);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Configure the button bindings
    configureButtonBindings();


  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    //Enable Wrist Moving Upwards
    Button.wristUp.whileTrue(m_wristRunUpwards);

    //Enable Wrist Moving Downwards
    Button.wristDown.whileTrue(m_wristRunDownwards);


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    return null;
    
  }
}