// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

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

    //Define Encoder Constants
        public static final int ENCODER_CPR = 42;


    //Define Drivetrain Constants

        //Define Motor Ports
        public static final int drivetrain_LEFT_FRONT_PORT = 3;
        public static final int drivetrain_LEFT_BACK_PORT = 1;
        public static final int drivetrain_RIGHT_FRONT_PORT = 4;
        public static final int drivetrain_RIGHT_BACK_PORT = 2;

        //Define Drivetrain Encoder Constants
        public static final double drivetrain_WHEEL_DIAMETER_METERS = 0.15;

        public static final double drivetrain_VELOCITY_CONVERSION_FACTOR = Math.PI * drivetrain_WHEEL_DIAMETER_METERS /(60 * 10.75);
        public static final double drivetrain_POSITION_CONVERSION_FACTOR = Math.PI * drivetrain_WHEEL_DIAMETER_METERS / 10.75;

        //Define Charge Station Balance Constants
        public static final double drivetrain_CHARGE_STATION_BALANCE_SPEED = 0.3;

    //Define Elevator Constants

        //Define Motor Ports (these are placeholders for now)
        public static final int elevator_LEFT_PORT = -314159;
        public static final int elevator_RIGHT_PORT = -100000000;

    //Define Intake Constants (currently placeholders)

        //Define Motor Ports
        public static final int intake_PIVOT_PORT = -1;
        public static final int intake_ROTATE_PORT = -2;

    //Define Limelight Constants

        public static final double limelight_TURN_ALIGNMENT_SPEED = 0.5;
        public static final double limelight_FORWARD_ALIGNMENT_SPEED = 0.4;

    //Define Autonomous Constants

        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;

        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(0.5842);

        public static final double ksVolts = 0.19712;
        public static final double kvVoltsSecondsPerMeter = 2.7996;
        public static final double kaVoltsSecondsSquaredPerMeter = 0.38673;
    
        //public static final double kPDriveVel = 3.6372;
        public static final double kPDriveVel = 3.0;
        
}
