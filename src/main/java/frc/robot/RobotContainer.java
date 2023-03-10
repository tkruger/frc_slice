// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.auto.AutoSelector;

import frc.robot.commands.*;
import frc.robot.commands.Drivetrain.*;
import frc.robot.commands.Elevator.*;
import frc.robot.commands.Intake.CalibrateMandiblesCommand;
import frc.robot.commands.Intake.RunMandiblesCommand;
import frc.robot.commands.LEDs.FlashColorCommand;
import frc.robot.commands.Wrist.*;
import frc.robot.commands.Limelight.*;
import frc.robot.commands.sequences.PickUpGamePieceDoubleSubstationSequence;
import frc.robot.commands.sequences.PickUpGamePieceGroundSequence;
import frc.robot.commands.sequences.PlaceConeHighRowSequence;
import frc.robot.commands.sequences.PlaceConeMidRowSequence;
import frc.robot.commands.sequences.PlaceCubeHighRowSequence;
import frc.robot.commands.sequences.PlaceCubeMidRowSequence;
import frc.robot.commands.sequences.PlaceGamePieceLowRowSequence;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final Drivetrain m_drivetrain = new Drivetrain();
  public final Elevator m_elevator = new Elevator();
  public final Wrist m_wrist = new Wrist();
  public final Intake m_intake = new Intake();
  public final Limelight m_limelight = new Limelight();
  public final ColorSensor m_colorSensor = new ColorSensor();
  public final LEDs m_leds = new LEDs();
  public final AutoSelector m_autoSelector = new AutoSelector(m_drivetrain, m_elevator, m_wrist, m_intake, m_colorSensor);

  public final Joystick leftJoystick = Button.leftJoystick;
  public final Joystick rightJoystick = Button.rightJoystick;
  public final Joystick manipulatorJoystick = Button.manipulatorJoystick;

  public final DrivetrainCommand m_oldDrive = new DrivetrainCommand(m_drivetrain, leftJoystick, rightJoystick);
  public final CurvatureDriveCommand m_curvatureDrive = new CurvatureDriveCommand(m_drivetrain, leftJoystick, rightJoystick);
  public final PIDDriveCommand m_PIDDrive = new PIDDriveCommand(m_drivetrain, leftJoystick, rightJoystick);
  public final ChargeStationBalanceCommand m_chargeStationBalance = new ChargeStationBalanceCommand(m_drivetrain);
  public final ChargeStationBalancePIDCommand m_ChargeStationBalancePID = new ChargeStationBalancePIDCommand(m_drivetrain);
  public final QuickTurnSequence m_quickTurn = new QuickTurnSequence(m_drivetrain);
  public final QuickTurnPIDCommand m_quickTurnPID = new QuickTurnPIDCommand(m_drivetrain);

  public final ElevatorRunCommand m_elevatorRunUpwards = new ElevatorRunCommand(m_elevator, true);
  public final ElevatorRunCommand m_elevatorRunDownwards = new ElevatorRunCommand(m_elevator, false);
  public final ElevatorJoystickRunCommand m_elevatorJoystickRun = new ElevatorJoystickRunCommand(m_elevator, manipulatorJoystick);
  public final CalibrateElevatorCommand m_calibrateElevator = new CalibrateElevatorCommand(m_elevator);
  public final ElevatorSetPIDCommand m_ElevatorSetPIDCommand = new ElevatorSetPIDCommand(m_elevator, 50);

  //public final SequentialCommandGroup m_wristRunUpwards = new WristRunCommand(m_wrist, true).andThen(new WristStationaryCommand(m_wrist));
  //public final SequentialCommandGroup m_wristRunDownwards = new WristRunCommand(m_wrist, false).andThen(new WristStationaryCommand(m_wrist));
  public final WristRunCommand m_wristRunUpwards = new WristRunCommand(m_wrist, true);
  public final WristRunCommand m_wristRunDownwards = new WristRunCommand(m_wrist, false);
  public final WristStationaryCommand m_wristStationary = new WristStationaryCommand(m_wrist);
  public final ResetAngleCommand m_resetWristAngle = new ResetAngleCommand(m_wrist);
  public final SetWristPosition m_testSetWristPosition = new SetWristPosition(m_wrist, -50);

  public final RunMandiblesCommand m_runMandiblesInwards = new RunMandiblesCommand(m_intake, true);
  public final RunMandiblesCommand m_runMandiblesOutwards = new RunMandiblesCommand(m_intake, false);
  public final CalibrateMandiblesCommand m_calibrateMandibles = new CalibrateMandiblesCommand(m_intake);

  public final LimelightAlignCommand m_limelightAlign = new LimelightAlignCommand(m_limelight, m_drivetrain);

  public final FlashColorCommand m_flashPurpleLEDs = new FlashColorCommand(m_leds, Color.kYellow, 0.5, 0.5);
  public final FlashColorCommand m_flashYellowLEDs = new FlashColorCommand(m_leds, Color.kPurple, 0.5, 0.5);

  public final PickUpGamePieceGroundSequence m_pickUpGamePieceGround = new PickUpGamePieceGroundSequence(m_elevator, m_wrist, m_intake, m_colorSensor);
  public final PickUpGamePieceDoubleSubstationSequence m_pickUpGamePieceDoubleSubstation = new PickUpGamePieceDoubleSubstationSequence(m_elevator, m_wrist, m_intake, m_colorSensor);
  public final PlaceGamePieceLowRowSequence m_placeGamePieceLowRow = new PlaceGamePieceLowRowSequence(m_elevator, m_wrist, m_intake);
  public final PlaceCubeMidRowSequence m_placeCubeMidRow = new PlaceCubeMidRowSequence(m_elevator, m_wrist, m_intake);
  public final PlaceConeMidRowSequence m_placeConeMidRow = new PlaceConeMidRowSequence(m_elevator, m_wrist, m_intake);
  public final PlaceCubeHighRowSequence m_placeCubeHighRow = new PlaceCubeHighRowSequence(m_elevator, m_wrist, m_intake);
  public final PlaceConeHighRowSequence m_placeConeHighRow = new PlaceConeHighRowSequence(m_elevator, m_wrist, m_intake);

  public final GoToStateCommand m_manualSetMidCube = new GoToStateCommand(m_elevator, m_wrist, Constants.States.MID_ROW_CUBE_STATE);
  public final GoToStateCommand m_manualSetHighCube = new GoToStateCommand(m_elevator, m_wrist, Constants.States.HIGH_ROW_CUBE_STATE);
  public final GoToStateCommand m_manualSetMidCone = new GoToStateCommand(m_elevator, m_wrist, Constants.States.MID_ROW_CONE_STATE);
  public final GoToStateCommand m_manualSetHighCone = new GoToStateCommand(m_elevator, m_wrist, Constants.States.HIGH_ROW_CONE_STATE);
  public final GoToStateCommand m_manualSetDoubleSubstation = new GoToStateCommand(m_elevator, m_wrist, Constants.States.DOUBLE_SUBSTATION_STATE);
  public final GoToStateCommand m_manualSetGround = new GoToStateCommand(m_elevator, m_wrist, Constants.States.LOW_ROW_GROUND_STATE);
  public final GoToStateCommand m_manualSetStowState = new GoToStateCommand(m_elevator, m_wrist, Constants.States.TRAVEL_STATE);

  public final ConditionalCommand m_setMidRowState = new ConditionalCommand(m_manualSetMidCone, m_manualSetMidCube, Button.setConeState);
  public final ConditionalCommand m_setHighRowState = new ConditionalCommand(m_manualSetHighCone, m_manualSetHighCube, Button.setConeState);

  //public final FlashColorCommand m_redLEDCommand = new FlashColorCommand(null, null, 0, 0)
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Configure the button bindings
    configureButtonBindings();

    m_drivetrain.setDefaultCommand(m_oldDrive);
    m_elevator.setDefaultCommand(m_elevatorJoystickRun);
    m_wrist.setDefaultCommand(m_wristStationary);
    m_intake.setDefaultCommand(new IdleCommand(m_intake));
    m_limelight.setDefaultCommand(new IdleCommand(m_limelight));
    m_colorSensor.setDefaultCommand(new IdleCommand(m_colorSensor));
    //m_leds.setDefaultCommand(new IdleCommand(m_leds));

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

    //Enable PID Charge Station Balance
    Button.chargeStationBalancePID.whileTrue(m_ChargeStationBalancePID);

    //Execute PID Drivetrain Quick Turn
    Button.quickTurnPID.onTrue(m_quickTurnPID);

    //Toggle Old Drive
    Button.oldDrive.toggleOnTrue(m_oldDrive);

    //Toggle PID Drive
    Button.PIDDrive.toggleOnTrue(m_PIDDrive);

    //Toggle Curvature Drive
    Button.curvatureDrive.toggleOnTrue(m_curvatureDrive);

    //Enable Limelight Alignment
    Button.limelightAlign.whileTrue(m_limelightAlign);

    //Execute Elevator Position Calibration
    Button.calibrateElevator.onTrue(m_calibrateElevator);

    //Enable Wrist Moving Upwards
    Button.wristUp.whileTrue(m_wristRunUpwards);

    //Enable Wrist Moving Downwards
    Button.wristDown.whileTrue(m_wristRunDownwards);

    //Execute Low Row Ground State Positioning
    Button.setLowRowGroundState.onTrue(m_placeGamePieceLowRow);

    //Execute Mid Row State Positioning
    Button.setMidRowState.onTrue(m_setMidRowState);

    //Execute High Row State Positioning
    Button.setHighRowState.onTrue(m_setHighRowState);

    //Execute Stow State Positioning
    Button.manualSetStowState.onTrue(m_manualSetStowState);

    //Enable Mandibles Moving Inwards
    Button.mandiblesInwards.whileTrue(m_runMandiblesInwards);

    //Enable Mandibles Moving Outwards
    Button.mandiblesOutwards.whileTrue(m_runMandiblesOutwards);

    //Execute Mandibles Position Calibration
    Button.calibrateMandibles.onTrue(m_calibrateMandibles);

    //Execute Wrist Angle Reset
    Button.resetWrist.onTrue(m_resetWristAngle);

    Button.rightButton8.onTrue(m_manualSetDoubleSubstation);
    Button.rightButton10.onTrue(m_manualSetHighCube);
    Button.rightButton12.onTrue(m_manualSetHighCone);
    Button.rightButton9.onTrue(m_manualSetMidCube);
    Button.rightButton11.onTrue(m_manualSetMidCone);

    //Toggle Purple LED Flashing
    Button.flashPurpleLEDs.toggleOnTrue(m_flashPurpleLEDs);

    //Toggle Yellow LED Flashing
    Button.flashYellowLEDs.toggleOnTrue(m_flashYellowLEDs);

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