// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public final class Button {

    //Define Joysticks
    public static Joystick leftJoystick = new Joystick(Constants.RobotContainer_LEFT_JOYSTICK_PORT);
    public static Joystick rightJoystick = new Joystick(Constants.RobotContainer_RIGHT_JOYSTICK_PORT);

    //Define Drivetrain Buttons
    public static Trigger driveMethod = new Trigger(() -> leftJoystick.getRawButtonPressed(10)); //Left Bottom 10

    //Define Indexer Buttons
    public static Trigger indexerUpFast = new Trigger(() -> rightJoystick.getRawButton(5)); //Right Top 5
    public static Trigger indexerDownFast = new Trigger(() -> leftJoystick.getRawButton(5)); //Left Top 5

    //Define Intake + Indexer Buttons
    public static Trigger indexerIntakeIn = new Trigger(() -> rightJoystick.getRawButton(3)); //Right Top 3
    public static Trigger indexerIntakeOut = new Trigger(() -> leftJoystick.getRawButton(3)); //Left Top 3

    //Define Intake Buttons
    public static Trigger intakeToggle = new Trigger(() -> rightJoystick.getRawButtonPressed(2)); //Right Top 2
    public static Trigger intakeReverse = new Trigger(() -> leftJoystick.getRawButton(2)); //Left Top 2

    //Define Shooter Buttons
    public static Trigger rightTrigger = new Trigger(() -> rightJoystick.getRawButton(1)); //Right Top 1
    public static Trigger leftTrigger = new Trigger(() -> leftJoystick.getRawButton(1)); //Left Top 1

    //Define Climber Buttons
    public static Trigger climberArmsDown = new Trigger(() -> rightJoystick.getRawButton(7)); //Right Bottom 7
    public static Trigger climberArmsUp = new Trigger(() -> rightJoystick.getRawButton(8)); //Right Bottom 8
    public static Trigger leftClimberDown = new Trigger(() -> rightJoystick.getRawButton(9)); //Right Bottom 9
    public static Trigger leftClimberUp = new Trigger(() -> rightJoystick.getRawButton(10)); //Right Bottom 10
    public static Trigger rightClimberDown = new Trigger(() -> rightJoystick.getRawButton(11)); //Right Bottom 11
    public static Trigger rightClimberUp = new Trigger(() -> rightJoystick.getRawButton(12)); //Right Bottom 12
    public static Trigger zeroClimber = new Trigger(() -> leftJoystick.getRawButtonPressed(9)); //Left Bottom 9

    //Define Pneumatics Buttons
    public static Trigger pneumaticsOut = new Trigger(() -> leftJoystick.getRawButton(7)); //Left Bottom 7
    public static Trigger pneumaticsIn = new Trigger(() -> leftJoystick.getRawButton(8)); //Left Bottom 8

    //Define Climber + Pneumatics Buttons
    public static Trigger climberPneumaticsBack = new Trigger(() -> leftJoystick.getRawButton(4)); //Left Top 4
    public static Trigger climberPneumaticsMin = new Trigger(() -> rightJoystick.getRawButton(4)); //Right Top 4
    public static Trigger climberPneumaticsUp = new Trigger(() -> rightJoystick.getRawButton(6)); //Left Top 6

    //Unassigned Right Joystick Buttons

    //Unassigned Left Joystick Buttons
    public static Trigger leftButton6 = new Trigger(() -> leftJoystick.getRawButton(6)); //Left Top 6
    public static Trigger leftButton11 = new Trigger(() -> leftJoystick.getRawButton(11)); //Left Bottom 11
    public static Trigger leftButton12 = new Trigger(() -> leftJoystick.getRawButton(12)); //Left Bottom 12
    

}
