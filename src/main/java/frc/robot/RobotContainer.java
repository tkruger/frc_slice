// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.auto.AutoSelector;
import frc.robot.commands.*;
import frc.robot.commands.Drivetrain.*;
import frc.robot.commands.Elevator.*;
import frc.robot.commands.Intake.*;
import frc.robot.commands.LEDs.*;
import frc.robot.commands.Wrist.*;
import frc.robot.commands.Limelight.*;
import frc.robot.commands.sequences.*;
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
  //public final ColorSensor m_colorSensor = new ColorSensor();
  public final LEDs m_leds = new LEDs();
  public final AutoSelector m_autoSelector = new AutoSelector(m_drivetrain, m_elevator, m_wrist, m_intake);

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
  public final ToggleForceVisionImplementation m_toggleForceVisionImplementation = new ToggleForceVisionImplementation(m_drivetrain);

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

  public final LimelightNodeAlignCommand m_nodeAlign = new LimelightNodeAlignCommand(m_limelight, m_drivetrain);
  public final DoubleSubstationAlignThenPickUpPieceSequence m_doubleSubstationAlignThenPickUpPiece = new DoubleSubstationAlignThenPickUpPieceSequence(m_drivetrain, m_limelight, m_elevator, m_wrist, m_intake);

  public final FlashColorCommand m_flashPurpleLEDs = new FlashColorCommand(m_leds, Color.kYellow, 5, 0.001);
  public final FlashColorCommand m_flashYellowLEDs = new FlashColorCommand(m_leds, Color.kPurple, 5, 0.001);
  public final RainbowLEDs m_idleLEDs = new RainbowLEDs(m_leds);

  //public final PickUpGamePieceGroundSequence m_pickUpGamePieceGround = new PickUpGamePieceGroundSequence(m_elevator, m_wrist, m_intake, m_colorSensor);
  //public final PickUpGamePieceDoubleSubstationSequence m_pickUpGamePieceDoubleSubstation = new PickUpGamePieceDoubleSubstationSequence(m_elevator, m_wrist, m_intake, m_colorSensor);
  public final PlaceGamePieceLowRowSequence m_placeGamePieceLowRow = new PlaceGamePieceLowRowSequence(m_elevator, m_wrist, m_intake);
  public final PlaceCubeMidRowSequence m_placeCubeMidRow = new PlaceCubeMidRowSequence(m_elevator, m_wrist, m_intake);
  public final PlaceConeMidRowSequence m_placeConeMidRow = new PlaceConeMidRowSequence(m_elevator, m_wrist, m_intake);
  public final PlaceHighRowSequence m_placeCubeHighRow = new PlaceHighRowSequence(m_elevator, m_wrist, m_intake);

  public final GoToStateCommand m_manualSetMidCube = new GoToStateCommand(m_elevator, m_wrist, Constants.States.MID_ROW_CUBE_STATE);
  public final ToHighRowSequence m_manualSetHighRow = new ToHighRowSequence(m_elevator, m_wrist);
  public final GoToStateCommand m_manualSetMidCone = new GoToStateCommand(m_elevator, m_wrist, Constants.States.MID_ROW_CONE_STATE);
  public final GoToStateCommand m_manualSetLowRowGround = new GoToStateCommand(m_elevator, m_wrist, Constants.States.LOW_ROW_GROUND_STATE);
  
  public final GoToStateCommand m_toDoubleSubstation = new GoToStateCommand(m_elevator, m_wrist, Constants.States.DOUBLE_SUBSTATION_STATE);
  public final ToDoubleSubstationSequence m_manualSetDoubleSubstation = new ToDoubleSubstationSequence(m_elevator, m_wrist);

  public final GoToStateCommand m_toStowTransitionState = new GoToStateCommand(m_elevator, m_wrist, Constants.States.TRANSITION_OUT_STATE);
  public final GoToStateCommand m_toStowSate = new GoToStateCommand(m_elevator, m_wrist, Constants.States.TRAVEL_STATE);
  public final StowSequence m_manualSetStow = new StowSequence(m_elevator, m_wrist);
  
  public final BrakeCommand m_brakeCommand = new BrakeCommand(m_drivetrain, true);
  public final BrakeCommand m_coastCommand = new BrakeCommand(m_drivetrain, false);

  //public final ConditionalCommand m_setMidRowState = new ConditionalCommand(m_manualSetMidCone, m_manualSetMidCube, Button.setConeState);
  //public final ConditionalCommand m_setHighRowState = new ConditionalCommand(m_manualSetHighCone, m_manualSetHighCube, Button.setConeState);
  public final TimedRunMandiblesCommand m_calibrateCloseMandibles = new TimedRunMandiblesCommand(m_intake, true, 0.3);
  public final SequentialCommandGroup m_calibrateCommands = new SequentialCommandGroup(m_calibrateCloseMandibles, m_calibrateElevator, m_resetWristAngle);
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
    //m_colorSensor.setDefaultCommand(new IdleCommand(m_colorSensor));
    m_leds.setDefaultCommand(m_idleLEDs);

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

    //Enable Node Alignment
    Button.nodeAlign.whileTrue(m_nodeAlign);

    //Toggle Vision Implementation Force
    Button.toggleForceVisionImplementation.onTrue(m_toggleForceVisionImplementation);

    //Enable Double Substation Align Then Pick Up Piece Sequence
    Button.doubleSubstationAlign.whileTrue(m_doubleSubstationAlignThenPickUpPiece);

    //Enable Wrist Moving Upwards
    Button.wristUp.whileTrue(m_wristRunUpwards);

    //Enable Wrist Moving Downwards
    Button.wristDown.whileTrue(m_wristRunDownwards);

    //Execute Low Row Ground State Positioning
    Button.toLowRowGroundState.onTrue(m_manualSetLowRowGround);

    //Execute Mid Row Cube State Positioning
    Button.toMidCubeState.onTrue(m_manualSetMidCube);

    //Execute Mid Row Cone State Positioning
    Button.toMidConeState.onTrue(m_manualSetMidCone);

    //Execute High Row State Positioning
    Button.toHighState.onTrue(m_manualSetHighRow);

    //Execute Stow State Positioning
    Button.toStowState.onTrue(m_manualSetStow);

    Button.toDoubleSubstationState.onTrue(m_manualSetDoubleSubstation);

    //Enable Mandibles Moving Inwards
    Button.mandiblesInwards.whileTrue(m_runMandiblesInwards);

    //Enable Mandibles Moving Outwards
    Button.mandiblesOutwards.whileTrue(m_runMandiblesOutwards);

    //Execute Mandibles Position Calibration
    Button.calibrateAll.onTrue(m_calibrateCommands);

    //Execute Wrist Angle Reset
    //Button.resetWrist.onTrue(m_resetWristAngle);

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

  public Command getBrakeCommand() {
    return m_brakeCommand;
  }

  public Command getCoastCommand() {
    return m_coastCommand;
  }
}