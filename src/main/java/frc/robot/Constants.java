// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public final class kJoysticks {

        public static final int LEFT_JOYSTICK_PORT = 0, RIGHT_JOYSTICK_PORT = 1;

        public static final double A_COEFFICIENT = 0.7;
        public static final double B_COEFFICIENT = 0.3;
        public static final int FIRST_POWER = 3;
        public static final int SECOND_POWER = 1;

    }

    public final class kEncoder {

        public static final int CPR = 1;

    }

    public static final class kDrivetrain {

        //These motor ports are placeholders for now
        public static final int 
        LEFT_FRONT_PORT_DRIVE = 5, 
        LEFT_FRONT_PORT_STEER = 6,
        LEFT_BACK_PORT_DRIVE = 1,
        LEFT_BACK_PORT_STEER = 2,
        RIGHT_FRONT_PORT_DRIVE = 7,
        RIGHT_FRONT_PORT_STEER = 8,
        RIGHT_BACK_PORT_DRIVE = 3,
        RIGHT_BACK_PORT_STEER = 4;

        //This is the diameter of kit of parts wheels and may be different for swerve
        public static final double WHEEL_DIAMETER_METERS = 0.15;
    
        public static final double LINEAR_VELOCITY_CONVERSION_FACTOR = Math.PI * WHEEL_DIAMETER_METERS /(60 * 10.75);
        public static final double ANGULAR_VELOCITY_CONVERSION_FACTOR = Math.PI * WHEEL_DIAMETER_METERS /(60 * 10.75) / (WHEEL_DIAMETER_METERS / 2);

        public static final double DISTANCE_CONVERSION_FACTOR = Math.PI * WHEEL_DIAMETER_METERS / 10.75;
        public static final double ANGLE_CONVERSION_FACTOR = Math.PI * WHEEL_DIAMETER_METERS / 10.75 / (WHEEL_DIAMETER_METERS / 2);

        /*These maximum velocities and accelerations are placeholders for now and should
        be slightly below the actual maximum velocity and acceleration that the robot is capable of*/
        public static final double kMaxVelocityMetersPerSecond = 4;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;

        //These are the track width and wheelbase length of the kit of parts drivetrain and may be different for swerve
        public static final double kTrackWidthMeters = 0.5842;
        public static final double kWheelbaseLengthMeters = 0.8128;

        //This constant uses the total length of the robot(0.8128 meters) for now as a placeholder for wheelbase length
        public static final double kMaxAngularVelocityRadiansPerSecond = 
            kMaxVelocityMetersPerSecond / Math.hypot(kTrackWidthMeters / 2.0, 0.8128 / 2.0);

        //This constant multiplies Pi by 2 as a placeholder for now
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI * 2;

        //This is a placeholder for now
        public static final double MAXIMUM_VOLTAGE = 10;

        public static final SwerveDriveKinematics kSwerveKinematics = new SwerveDriveKinematics(
            // Front left
            new Translation2d(kTrackWidthMeters / 2.0, kWheelbaseLengthMeters / 2.0),
            // Back left
            new Translation2d(-kTrackWidthMeters / 2.0, kWheelbaseLengthMeters / 2.0),
            // Front right
            new Translation2d(kTrackWidthMeters  / 2.0, -kWheelbaseLengthMeters / 2.0),
            // Back right
            new Translation2d(-kTrackWidthMeters  / 2.0, -kWheelbaseLengthMeters / 2.0));

    }

    public final class kAutonomous {
    
        //These constants are currently placeholders and need to be determined by characterization
        public static final double ksVolts = 0.19712;
        public static final double kvVoltsSecondsPerMeter = 2.7996;
        public static final double kaVoltsSecondsSquaredPerMeter = 0.38673;

        //This constant is currently a placeholder and needs to be determined by characterization
        public static final double kPDriveVel = 3.6372;

        // Ramsete controller parameters. There are maethematically determined to be the best parameters for almost every robot
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;

    }

}