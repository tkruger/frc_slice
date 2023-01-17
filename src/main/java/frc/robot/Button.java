// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public final class Button {

    //Define Joysticks
    public static Joystick leftJoystick = new Joystick(Constants.RobotContainer_LEFT_JOYSTICK_PORT);
    public static Joystick rightJoystick = new Joystick(Constants.RobotContainer_RIGHT_JOYSTICK_PORT);

    //Define Drivetrain Buttons
    public static Trigger driveMethod = new JoystickButton(leftJoystick, 10); //Left Bottom 10

    //Define Indexer Buttons
    public static Trigger indexerUpFast = new JoystickButton(rightJoystick, 5); //Right Top 5
    public static Trigger indexerDownFast = new JoystickButton(leftJoystick, 5); //Left Top 5

    //Define Intake + Indexer Buttons
    public static Trigger indexerIntakeIn = new JoystickButton(rightJoystick, 3); //Right Top 3
    public static Trigger indexerIntakeOut = new JoystickButton(leftJoystick, 3); //Left Top 3

    //Define Intake Buttons
    public static Trigger intakeToggle = new JoystickButton(rightJoystick, 2); //Right Top 2
    public static Trigger intakeReverse = new JoystickButton(leftJoystick, 2); //Left Top 2

    //Define Shooter Buttons
    public static Trigger rightTrigger = new JoystickButton(rightJoystick, 1); //Right Top 1
    public static Trigger leftTrigger = new JoystickButton(leftJoystick, 1);; //Left Top 1

    //Define Climber Buttons
    public static Trigger climberArmsDown = new JoystickButton(rightJoystick, 7); //Right Bottom 7
    public static Trigger climberArmsUp = new JoystickButton(rightJoystick, 8); //Right Bottom 8
    public static Trigger leftClimberDown = new JoystickButton(rightJoystick, 9); //Right Bottom 9
    public static Trigger leftClimberUp = new JoystickButton(rightJoystick, 10); //Right Bottom 10
    public static Trigger rightClimberDown = new JoystickButton(rightJoystick, 11); //Right Bottom 11
    public static Trigger rightClimberUp = new JoystickButton(rightJoystick, 12); //Right Bottom 12
    public static Trigger zeroClimber = new JoystickButton(leftJoystick, 9); //Left Bottom 9

    //Define Pneumatics Buttons
    public static Trigger pneumaticsOut = new JoystickButton(leftJoystick, 7); //Left Bottom 7
    public static Trigger pneumaticsIn = new JoystickButton(leftJoystick, 8); //Left Bottom 8

    //Define Climber + Pneumatics Buttons
    public static Trigger climberPneumaticsBack = new JoystickButton(leftJoystick, 4); //Left Top 4
    public static Trigger climberPneumaticsMin = new JoystickButton(rightJoystick, 4); //Right Top 4
    public static Trigger climberPneumaticsUp = new JoystickButton(rightJoystick, 6); //Right Top 6

    //Unassigned Right Joystick Buttons

    //Unassigned Left Joystick Buttons
    public static Trigger leftButton6 = new JoystickButton(leftJoystick, 6); //Left Top 6
    public static Trigger leftButton11 = new JoystickButton(leftJoystick, 11); //Left Bottom 11
    public static Trigger leftButton12 = new JoystickButton(leftJoystick, 12); //Left Bottom 12
    
}
