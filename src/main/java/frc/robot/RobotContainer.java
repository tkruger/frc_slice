// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.commands.*;
import frc.robot.commands.Pneumatics.PneumaticsIdleCommand;
import frc.robot.commands.Pneumatics.PneumaticsInCommand;
import frc.robot.commands.Pneumatics.PneumaticsOutCommand;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final Pneumatics m_pneumatics = new Pneumatics();

  private static Joystick leftJoystick = Button.leftJoystick;
  private static Joystick rightJoystick = Button.rightJoystick;

  //Create any commands here
  private final DriveCommand m_drive = new DriveCommand(m_drivetrain);

  private final PneumaticsIdleCommand m_pneumaticsIdleCmd = new PneumaticsIdleCommand(m_pneumatics);
  private final PneumaticsInCommand m_pneumaticsInCmd = new PneumaticsInCommand(m_pneumatics);
  private final PneumaticsOutCommand m_pneumaticsOutCmd = new PneumaticsOutCommand(m_pneumatics);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    //Setting default commands
    m_drivetrain.setDefaultCommand(m_drive);
    m_pneumatics.setDefaultCommand(m_pneumaticsIdleCmd);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Call any buttons here along with the commands you want to run when they are pressed

    //pneumatics in and pneumatics out commands
    Button.rightButton1.whenPressed(m_pneumaticsInCmd);
    Button.rightButton2.whenPressed(m_pneumaticsOutCmd);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // A DriveCommand will run in autonomous
    return m_drive;
  }
}
