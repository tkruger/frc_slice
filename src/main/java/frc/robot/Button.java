// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public final class Button {

    //Define Input Devices
    public static Joystick leftJoystick = new Joystick(Constants.InputDevices.LEFT_JOYSTICK_PORT);
    public static Joystick rightJoystick = new Joystick(Constants.InputDevices.RIGHT_JOYSTICK_PORT);
    public static CommandGenericHID manipulatorController = new CommandGenericHID(Constants.InputDevices.MANIPULATOR_CONTROLLER_PORT);

    //Define Drivetrain Buttons
    public static Trigger oldDrive = new JoystickButton(leftJoystick, 7); //Left Bottom 7
    public static Trigger PIDDrive = new JoystickButton(leftJoystick, 8); //Left Bottom 8
    public static Trigger curvatureDrive = new JoystickButton(leftJoystick, 9); //Left Bottom 9
    public static Trigger chargeStationBalance = new JoystickButton(leftJoystick, 6); //Left Top 6
    public static Trigger chargeStationBalancePID = new JoystickButton(leftJoystick, 5); //Left Top 5
    public static Trigger quickTurnPID1 = new JoystickButton(leftJoystick, 1); //Left Top 1
    public static Trigger quickTurnPID2 = new JoystickButton(rightJoystick, 1); //Right Top 1
    public static Trigger quickTurnPID = quickTurnPID1.and(quickTurnPID2); //Left Top 1 and Right Top 1
    public static Trigger slowMode = new JoystickButton(rightJoystick, 2); //Right Top 2

    //Define Elevator Buttons
    public static Trigger calibrateElevator = new JoystickButton(rightJoystick, 6); //Right Top 6

    //Define Wrist Buttons
    public static Trigger resetWrist = new JoystickButton(rightJoystick, 4); //Right Top 4
    public static Trigger runWristController = new Trigger(() -> manipulatorController.getRawAxis(3) != 0);

    //Define Intake Buttons
    public static Trigger mandiblesInwards = manipulatorController.button(5); //Manipulator Left Bumper
    public static Trigger mandiblesOutwards = manipulatorController.button(6); //Manipulator Right Bumper
    public static Trigger calibrateAll = manipulatorController.button(7); //Manipulator Left Trigger

    //Define Limelight Buttons
    public static Trigger xAlign = new JoystickButton(rightJoystick, 8); //Right Bottom 8

    //Define LED Buttons
    public static Trigger flashPurpleLEDs = new JoystickButton(rightJoystick, 5); //Right Top 5
    public static Trigger flashYellowLEDs = new JoystickButton(rightJoystick, 3); //Right Top 3

    //Define RobotState Buttons
    public static Trigger toHighState = manipulatorController.button(4); //Manipulator Y Button
    public static Trigger toMidCubeState = manipulatorController.button(1); //Manipulator X Button
    public static Trigger toMidConeState = manipulatorController.button(3); //Manipulator B Button
    public static Trigger toLowRowGroundState = manipulatorController.button(2); //Manipulator A Button
    public static Trigger toDoubleSubstationState = manipulatorController.pov(0); //Manipulator D Pad Up
    public static Trigger toStowState = manipulatorController.button(8); //Manipulator Right Trigger

    //Define Trajectory Alignment Buttons
    public static Trigger doubleSubstationAlignAndPosition = new JoystickButton(leftJoystick, 3); //Left Top 3
    public static Trigger nodeAlignAndPosition = new JoystickButton(leftJoystick, 4); //Left Top 4

    //Unassigned Right Joystick Buttons
    public static Trigger rightButton9 = new JoystickButton(rightJoystick, 9); //Right Bottom 9
    public static Trigger rightButton10 = new JoystickButton(rightJoystick, 10); //Right Bottom 10
    public static Trigger rightButton11 = new JoystickButton(rightJoystick, 11); //Right Bottom 11
    public static Trigger rightButton12 = new JoystickButton(rightJoystick, 12); //Right Bottom 12

    //Unassigned Right Joystick POV Axes
    public static POVButton rightMiniJoystickUp = new POVButton(rightJoystick, 0);
    public static POVButton rightMiniJoystickUpRight = new POVButton(rightJoystick, 45);
    public static POVButton rightMiniJoystickRight = new POVButton(rightJoystick, 90);
    public static POVButton rightMiniJoystickDownRight = new POVButton(rightJoystick, 135);
    public static POVButton rightMiniJoystickDown = new POVButton(rightJoystick, 180);
    public static POVButton rightMiniJoystickDownLeft = new POVButton(rightJoystick, 225);
    public static POVButton rightMiniJoystickLeft = new POVButton(rightJoystick, 270);
    public static POVButton rightMiniJoystickUpLeft = new POVButton(rightJoystick, 315);

    //Unassigned Left Joystick Buttons
    public static Trigger leftButton2 = new JoystickButton(leftJoystick, 2); //Left Top 2
    public static Trigger leftButton10 = new JoystickButton(leftJoystick, 10); //Left Bottom 10
    public static Trigger leftButton11 = new JoystickButton(leftJoystick, 11); //Left Bottom 11
    public static Trigger leftButton12 = new JoystickButton(leftJoystick, 12); //Left Bottom 12

    //Unassigned Left Joystick POV Axes
    public static POVButton leftMiniJoystickUp = new POVButton(leftJoystick, 0);
    public static POVButton leftMiniJoystickUpRight = new POVButton(leftJoystick, 45);
    public static POVButton leftMiniJoystickRight = new POVButton(leftJoystick, 90);
    public static POVButton leftMiniJoystickDownRight = new POVButton(leftJoystick, 135);
    public static POVButton leftMiniJoystickDown = new POVButton(leftJoystick, 180);
    public static POVButton leftMiniJoystickDownLeft = new POVButton(leftJoystick, 225);
    public static POVButton leftMiniJoystickLeft = new POVButton(leftJoystick, 270);
    public static POVButton leftMiniJoystickUpLeft = new POVButton(leftJoystick, 315);

    //Unassigned Manipulator Controller Buttons
    public static Trigger manipulatorButton9 = manipulatorController.button(9); //Manipulator Back
    public static Trigger manipulatorButton10 = manipulatorController.button(10); //Manipulator Start
    public static Trigger manipulatorButton11 = manipulatorController.button(11); //Manipulator Left Stick Push
    public static Trigger manipulatorButton12 = manipulatorController.button(12); //Manipulator Right Stick Push

    //Unassigned Manipulator Controller POV Axes
    public static Trigger manipulatorMiniJoystickUpRight = manipulatorController.pov(45);
    public static Trigger manipulatorMiniJoystickRight = manipulatorController.pov(90);
    public static Trigger manipulatorMiniJoystickDownRight = manipulatorController.pov(135);
    public static Trigger manipulatorMiniJoystickDown = manipulatorController.pov(180);
    public static Trigger manipulatorMiniJoystickDownLeft = manipulatorController.pov(225);
    public static Trigger manipulatorMiniJoystickLeft = manipulatorController.pov(270);
    public static Trigger manipulatorMiniJoystickUpLeft = manipulatorController.pov(315);

}
