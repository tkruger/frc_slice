// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DigitalSource;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final int INTAKE_INTAKE_MOTOR = 11;

    public static final int LEFT_JOYSTICK = 0;
    public static final int RIGHT_JOYSTICK = 1;

    public static final int CANSPARKMAX_LEFT_MOTOR_FRONT = 3;
    public static final int CANSPARKMAX_LEFT_MOTOR_BACK = 1;
    public static final int CANSPARKMAX_RIGHT_MOTOR_FRONT = 4;
    public static final int CANSPARKMAX_RIGHT_MOTOR_BACK = 2;

    public static final int INDEXER_INDEXER_MOTOR = 7;

    public static final int[] DRIVETRAIN_FRONT_LEFT_ENCODER = new int[] {0, 1};
    public static final int[] DRIVETRAIN_BACK_LEFT_ENCODER = new int[] {2, 3};
    public static final int[] DRIVETRAIN_FRONT_RIGHT_ENCODER = new int[] {4, 5};
    public static final int[] DRIVETRAIN_BACK_RIGHT_ENCODER = new int[] {6, 7};

    public static final double[] DRIVETRAIN_LEFT_ENCODERS = new double[] {};

    public static final boolean DRIVETRAIN_FRONT_LEFT_ENCODER_REVERSED = false;
    public static final boolean DRIVETRAIN_BACK_LEFT_ENCODER_REVERSED = true;
    public static final boolean DRIVETRAIN_FRONT_RIGHT_ENCODER_REVERSED = false;
    public static final boolean DRIVETRAIN_BACK_RIGHT_ENCODER_REVERSED = true;

    public static final double DRIVETRAIN_ENCODER_DISTANCE_PER_PULSE = 0;

    public static final int DRIVETRAIN_ENCODER_CPR = 1024;
    public static final double DRIVETRAIN_WHEEL_DIAMETER_METERS = 0.15;

    public static final double kEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (DRIVETRAIN_WHEEL_DIAMETER_METERS * Math.PI) / (double) DRIVETRAIN_ENCODER_CPR;

}
