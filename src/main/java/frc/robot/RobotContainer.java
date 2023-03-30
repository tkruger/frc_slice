// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.auto.AutoSelector;
import frc.robot.commands.Drivetrain.*;
import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Joystick;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private static Joystick leftJoystick = Button.leftJoystick;
  private static Joystick rightJoystick = Button.rightJoystick;

  // The robot's subsystems and commands are defined here...
  public final SwerveDrivetrain m_swerveDrivetrain = new SwerveDrivetrain();

  public final AutoSelector m_autoSelector = new AutoSelector(m_swerveDrivetrain);

  public final SwerveDriveCommand m_swerveDrive = new SwerveDriveCommand(m_swerveDrivetrain, leftJoystick, rightJoystick);
  public final SwerveDrivePIDCommand m_swerveDrivePID = new SwerveDrivePIDCommand(m_swerveDrivetrain, leftJoystick, rightJoystick);

  public final BrakeCommand m_brakeCommand = new BrakeCommand(m_swerveDrivetrain, true);
  public final BrakeCommand m_coastCommand = new BrakeCommand(m_swerveDrivetrain, false);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Configure the button bindings
    configureButtonBindings();

    m_swerveDrivetrain.setDefaultCommand(m_swerveDrive);

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoSelector.getAutoMode();
  }

  public Command getBrakeCommand() {
    return m_brakeCommand;
  }

  public Command getCoastCommand() {
    return m_coastCommand;
  }

}
