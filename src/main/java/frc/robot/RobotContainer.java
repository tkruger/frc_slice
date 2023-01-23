// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.Drivetrain.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj.Joystick;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final Drivetrain m_drivetrain = new Drivetrain();

  public final Joystick leftJoystick = Button.leftJoystick;
  public final Joystick rightJoystick = Button.rightJoystick;

  public final DrivetrainCommand m_oldDrivetrain = new DrivetrainCommand(m_drivetrain, leftJoystick, rightJoystick); 
  public final PIDDriveCommand m_PIDDrive = new PIDDriveCommand(m_drivetrain, leftJoystick, rightJoystick);
  public final ChargeStationBalanceCommand m_chargeStationBalance = new ChargeStationBalanceCommand(m_drivetrain);
  public final QuickTurnCommand m_quickTurn = new QuickTurnCommand(m_drivetrain);

  public final AutoSelector m_autoSelector = new AutoSelector(m_drivetrain);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Configure the button bindings
    configureButtonBindings();

    m_drivetrain.setDefaultCommand(new CurvatureDriveCommand(m_drivetrain, leftJoystick, rightJoystick));

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    //Enable Charge Station Balance
    Button.chargeStationBalance.whileTrue(m_chargeStationBalance);

    //Execute Drivetrain Quick Turn
    Button.quickTurn.onTrue(m_quickTurn);

    //Toggle Drive Mode
    Button.driveMethod.toggleOnTrue(m_PIDDrive);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autoSelector.getAutoMode();
  }
}