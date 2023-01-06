// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.auto.*;
import frc.robot.commands.Drivetrain.*;
import frc.robot.commands.Limelight.*;
import frc.robot.commands.Pneumatics.*;
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
  static final Drivetrain m_drivetrain = new Drivetrain();
  static final Limelight m_limelight = new Limelight();
  static final Pneumatics m_pneumatics = new Pneumatics();

  private static Joystick leftJoystick = Button.leftJoystick;
  public static Joystick rightJoystick = Button.rightJoystick;
  //static final RamseteCommand m_testTrajectory = Trajectories.generateRamseteCommand(m_drivetrain, Trajectories.testTrajectory);

  static final LimelightScheduleableCommand m_limelightAlign = new LimelightScheduleableCommand(m_limelight, m_drivetrain);
 
  static final PneumaticsInCommand m_inPneumatics = new PneumaticsInCommand(m_pneumatics);
  static final PneumaticsOutCommand m_outPneumatics = new PneumaticsOutCommand(m_pneumatics);

  static final DrivetrainCommand m_oldDrivetrain = new DrivetrainCommand(m_drivetrain, leftJoystick, rightJoystick); 
  static final PIDDriveCommand m_PIDDrive = new PIDDriveCommand(m_drivetrain, leftJoystick, rightJoystick);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    Paths.createAutoPaths();

    // Configure the button bindings
    configureButtonBindings();

    m_drivetrain.setDefaultCommand(new CurvatureDriveCommand(m_drivetrain, leftJoystick, rightJoystick));

    m_limelight.setDefaultCommand(new LimelightIdleCommand(m_limelight));
    m_pneumatics.setDefaultCommand(new PneumaticsIdleCommand(m_pneumatics));

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //Toggle Pneumatics
    Button.pneumaticsIn.whileTrue(m_inPneumatics);
    Button.pneumaticsOut.whileTrue(m_outPneumatics);

    //Button.driveMethod.toggleOnTrue(m_PIDDrive);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //return new TrajectoryFollowerSequence(m_drivetrain, Trajectories.testTrajectory, Trajectories.testTrajectory.getInitialPose());
    //return new simpleAutoRoutine(m_indexer, m_intake, m_shooter, m_drivetrain, m_limelight, leftJoystick, rightJoystick);
    return new trajectoryFollowerAutoRoutine(m_drivetrain, m_limelight, leftJoystick, rightJoystick);
  }
}