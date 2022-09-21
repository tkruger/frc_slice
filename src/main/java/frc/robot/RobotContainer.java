// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.auto.*;
import frc.robot.commands.Drivetrain.*;
import frc.robot.commands.Indexer.*;
import frc.robot.commands.Shooter.*;
import frc.robot.commands.Intake.*;
import frc.robot.commands.Limelight.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Joystick;
//import frc.robot.Button;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static final Drivetrain m_drivetrain = new Drivetrain();
  public static final Shooter m_shooter = new Shooter();
  public static final Indexer m_indexer = new Indexer();
  public static final Intake m_intake = new Intake();
  public static final Limelight m_limelight = new Limelight();

  private static Joystick leftJoystick = Button.leftJoystick;
  private static Joystick rightJoystick = Button.rightJoystick;

  public static final alignlessShootSequence m_alignlessShootAuto = 
        new alignlessShootSequence(m_indexer, m_shooter, leftJoystick, rightJoystick);

  public static final alignedShootSequence m_alignedShootAuto = 
        new alignedShootSequence(m_indexer, m_shooter, m_drivetrain, m_limelight, leftJoystick, rightJoystick);

  public static final ShooterCommand m_shooterCommand =
        new ShooterCommand(m_shooter, leftJoystick, rightJoystick);

  public static final LimelightScheduleableCommand m_limelightAlign = new LimelightScheduleableCommand(m_limelight, m_drivetrain);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Configure the button bindings
    configureButtonBindings();

    m_drivetrain.setDefaultCommand(new DrivetrainCommand(m_drivetrain, leftJoystick, rightJoystick));
    m_indexer.setDefaultCommand(new IndexerCommand(m_indexer, leftJoystick, rightJoystick));
    m_shooter.setDefaultCommand(new ShooterCommand(m_shooter, leftJoystick, rightJoystick));
    m_intake.setDefaultCommand(new IntakeCommand(m_intake, leftJoystick, rightJoystick));
    m_limelight.setDefaultCommand(new LimelightIdleCommand(m_limelight));

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //Shoot without aligning
    Button.rightButton12.whenHeld(m_alignlessShootAuto);

    //Align without shooting
    Button.leftTrigger.whenHeld(m_limelightAlign);

    //Align and shoot
    Button.rightTrigger.whenHeld(m_alignedShootAuto);

    //Spin flywheels
    Button.leftButton12.whenPressed(m_shooterCommand);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // A alignlessShootAuto will run in autonomous
    return m_alignedShootAuto;
  }
}
