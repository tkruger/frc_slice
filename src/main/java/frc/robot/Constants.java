// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final int 
    drivetrain_LEFT_FRONT_PORT_DRIVE = 5, 
    drivetrain_LEFT_FRONT_PORT_STEER = 6,
    drivetrain_LEFT_BACK_PORT_DRIVE = 1,
    drivetrain_LEFT_BACK_PORT_STEER = 2,
    drivetrain_RIGHT_FRONT_PORT_DRIVE = 7,
    drivetrain_RIGHT_FRONT_PORT_STEER = 8,
    drivetrain_RIGHT_BACK_PORT_DRIVE = 3,
    drivetrain_RIGHT_BACK_PORT_STEER = 4;

    public static final int drivetrain_ENCODER_CPR = 42;
    public static final double drivetrain_WHEEL_DIAMETER_METERS = 0.15;
    
    public static final double drivetrain_VELOCITY_CONVERSION_FACTOR = Math.PI * drivetrain_WHEEL_DIAMETER_METERS /(60 * 10.75);
    public static final double drivetrain_POSITION_CONVERSION_RATIO = Math.PI * drivetrain_WHEEL_DIAMETER_METERS / 10.75;

    public static final int RobotContainer_LEFT_JOYSTICK_PORT = 0, RobotContainer_RIGHT_JOYSTICK_PORT = 1;
    
    public static final double kTrackWidthMeters = 0.5842;

    public static final double kMaxSpeedMetersPerSeconds = 1.0;
    public static final double kMaxAccelerationMetersPerSecondSquared = 0.5;

    //This constant uses the total length of the robot(0.8128 meters) for now as a placeholder for wheelbase length
    public static final double kMaxAngularVelocityRadiansPerSecond = 
        kMaxSpeedMetersPerSeconds / Math.hypot(kTrackWidthMeters / 2.0, 0.8128 / 2.0);

    // ===============================
    // Autonomous
    // ===============================
    public static final double ksVolts = 0.19712;
    public static final double kvVoltsSecondsPerMeter = 2.7996;
    public static final double kaVoltsSecondsSquaredPerMeter = 0.38673;

    public static final double kPDriveVel = 3.6372;

    // Ramsete controller parameters. There are maethematically determined to be the best parameters for almost every robot
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

}