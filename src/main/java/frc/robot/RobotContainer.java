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
import edu.wpi.first.wpilibj2.command.RamseteCommand;
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
  static final Shooter m_shooter = new Shooter();
  static final Indexer m_indexer = new Indexer();
  static final Intake m_intake = new Intake();
  static final Limelight m_limelight = new Limelight();

  private static Joystick leftJoystick = Button.leftJoystick;
  private static Joystick rightJoystick = Button.rightJoystick;

  static final alignlessShootSequence m_alignlessShootAuto = 
        new alignlessShootSequence(m_indexer, m_shooter, leftJoystick, rightJoystick);

  static final alignedShootSequence m_alignedShootAuto = 
        new alignedShootSequence(m_indexer, m_shooter, m_drivetrain, m_limelight, leftJoystick, rightJoystick);

  static final smartShootSequence m_smartShootAuto = 
        new smartShootSequence(m_indexer, m_shooter, m_drivetrain, m_limelight, leftJoystick, rightJoystick);

  static final RamseteCommand m_testTrajectory = Trajectories.generateRamseteCommand(m_drivetrain, Trajectories.testTrajectory);

  static final LimelightScheduleableCommand m_limelightAlign = new LimelightScheduleableCommand(m_limelight, m_drivetrain);

  static final IndexerSchedulableCommand m_runIndexerUp = new IndexerSchedulableCommand(m_indexer, -0.5);
  static final IndexerSchedulableCommand m_runIndexerDown = new IndexerSchedulableCommand(m_indexer, 0.5);
 
  static final IntakeForwardCommand m_forwardIntake = new IntakeForwardCommand(m_intake);
  static final IntakeReverseCommand m_reverseIntake = new IntakeReverseCommand(m_intake);

  static final IndexerIntakeCommand m_inIntakeIndexer = new IndexerIntakeCommand(m_indexer, m_intake, true);
  static final IndexerIntakeCommand m_outIntakeIndexer = new IndexerIntakeCommand(m_indexer, m_intake, false);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Configure the button bindings
    configureButtonBindings();

    m_drivetrain.setDefaultCommand(new DrivetrainCommand(m_drivetrain, leftJoystick, rightJoystick));
    m_indexer.setDefaultCommand(new IndexerSchedulableCommand(m_indexer, 0));
    m_shooter.setDefaultCommand(new ShooterIdleCommand(m_shooter));
    m_intake.setDefaultCommand(new IntakeIdleCommand(m_intake));
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
    Button.rightTrigger.whenHeld(m_alignlessShootAuto);

    //Any* distance shooting
    Button.leftTrigger.whenHeld(m_smartShootAuto);

    //Run indexer up
    Button.indexerUpFast.whenHeld(m_runIndexerUp);
    
    //Run indexer down
    Button.indexerDownFast.whenHeld(m_runIndexerDown);

    //Toggle intake
    Button.intakeToggle.toggleWhenPressed(m_forwardIntake);

    //Reverse intake
    Button.intakeReverse.whenHeld(m_reverseIntake);

    Button.indexerIntakeIn.whenHeld(m_inIntakeIndexer);
    Button.indexerIntakeOut.whenHeld(m_outIntakeIndexer);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_testTrajectory;
  }
}