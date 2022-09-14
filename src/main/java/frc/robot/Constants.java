// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    //Define Joystick Ports
    public static final int RobotContainer_LEFT_JOYSTICK_PORT = 0;
    public static final int RobotContainer_RIGHT_JOYSTICK_PORT = 1;

    //Define Drivetrain Motor Ports
    public static final int drivetrain_LEFT_FRONT_PORT = 3;
    public static final int drivetrain_LEFT_BACK_PORT = 1;
    public static final int drivetrain_RIGHT_FRONT_PORT = 4;
    public static final int drivetrain_RIGHT_BACK_PORT = 2;

    //Define Shooter Motor Ports
    public static final int shooter_FLYWHEEL_PRIMARY_PORT = 9;
    public static final int shooter_FLYWHEEL_SECONDARY_PORT = 6;

    //Define Intake Motor Port
    public static final int intake_MOTOR_PORT = 11;

    //Define Indexer Motor Port
    public static final int indexer_MOTOR_PORT = 10;

    //Define Encoder Ports
    public static final int[] drivetrain_LEFT_ENCODER = new int[] {0,1};
    public static final int[] drivetrain_RIGHT_ENCODER = new int[] {2,3};

    //Define Encoder Reveresed Booleans
    public static final boolean drivetrain_LEFT_ENCODER_REVERSED = false;
    public static final boolean drivetrain_RIGHT_ENCODER_REVERSED = false;

    //Define Encoder Wheel Information
    public static final int drivetrain_ENCODER_CPR = 1024;
    public static final double drivetrain_WHEEL_DIAMETER_METERS = 0.15;

    //Define Encoder Distance Calculation Math
    public static final double drivetrain_ENCODER_DISTANCE_PER_PULSE =
        // Assumes the encoders are directly mounted on the wheel shafts
        (drivetrain_WHEEL_DIAMETER_METERS * Math.PI) / (double) drivetrain_ENCODER_CPR;


    // ===============================
    // Autonomous
    // ===============================
    public static final double ksVolts = 0.19712;
    public static final double kvVoltsSecondsPerMeter = 2.7996;
    public static final double kaVoltsSecondsSquaredPerMeter = 0.38673;

    public static final double kPDriveVel = 3.6372;

    public static final double kTrackWidthMeters = 0.5842;
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackWidthMeters);

    public static final double kMaxSpeedMetersPerSeconds = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;

    // Ramsete controller parameters. There are maethematically determined to be the best parameters for almost every robot
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
    }
