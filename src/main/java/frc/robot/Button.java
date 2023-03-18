// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public final class Button {

    //Define Joysticks
    public static Joystick leftJoystick = new Joystick(Constants.Joysticks.LEFT_JOYSTICK_PORT);
    public static Joystick rightJoystick = new Joystick(Constants.Joysticks.RIGHT_JOYSTICK_PORT);

    //Unassigned Left Joystick Buttons
    public static Trigger leftButton1 = new JoystickButton(leftJoystick, 1); //Left Top 1
    public static Trigger leftButton2 = new JoystickButton(leftJoystick, 1); //Left Top 2
    public static Trigger leftButton3 = new JoystickButton(leftJoystick, 1); //Left Top 3
    public static Trigger leftButton4 = new JoystickButton(leftJoystick, 1); //Left Top 4
    public static Trigger leftButton5 = new JoystickButton(leftJoystick, 1); //Left Top 5
    public static Trigger leftButton6 = new JoystickButton(leftJoystick, 1); //Left Top 6
    public static Trigger leftButton7 = new JoystickButton(leftJoystick, 1); //Left Bottom 7
    public static Trigger leftButton8 = new JoystickButton(leftJoystick, 1); //Left Bottom 8
    public static Trigger leftButton9 = new JoystickButton(leftJoystick, 1); //Left Bottom 9
    public static Trigger leftButton10 = new JoystickButton(leftJoystick, 1); //Left Bottom 10
    public static Trigger leftButton11 = new JoystickButton(leftJoystick, 1); //Left Bottom 11
    public static Trigger leftButton12 = new JoystickButton(leftJoystick, 1); //Left Bottom 12

    //Unassigned Right Joystick Buttons
    public static Trigger rightButton1 = new JoystickButton(rightJoystick, 1); //Right Top 1
    public static Trigger rightButton2 = new JoystickButton(rightJoystick, 1); //Right Top 2
    public static Trigger rightButton3 = new JoystickButton(rightJoystick, 1); //Right Top 3
    public static Trigger rightButton4 = new JoystickButton(rightJoystick, 1); //Right Top 4
    public static Trigger rightButton5 = new JoystickButton(rightJoystick, 1); //Right Top 5
    public static Trigger rightButton6 = new JoystickButton(rightJoystick, 1); //Right Top 6
    public static Trigger rightButton7 = new JoystickButton(rightJoystick, 1); //Right Bottom 7
    public static Trigger rightButton8 = new JoystickButton(rightJoystick, 1); //Right Bottom 8
    public static Trigger rightButton9 = new JoystickButton(rightJoystick, 1); //Right Bottom 9
    public static Trigger rightButton10 = new JoystickButton(rightJoystick, 1); //Right Bottom 10
    public static Trigger rightButton11 = new JoystickButton(rightJoystick, 1); //Right Bottom 11
    public static Trigger rightButton12 = new JoystickButton(rightJoystick, 1); //Right Bottom 12    

}