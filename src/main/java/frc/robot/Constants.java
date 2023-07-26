// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SerialPort.Port;

import frc.lib.config.SwerveModuleConstants;

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

    /**
     * ALL CONSTANTS IN THIS CLASS ARE PLACEHOLDERS FOR NOW
     */
    public static final class kDrivetrain {

        public static final Port NAVX_PORT = Port.kUSB1;
        public static final boolean INVERT_GYRO = true; // Always ensure Gyro is CCW+ CW-
    
        /* Drivetrain Constants */
        public static final double TRACK_WIDTH = Units.inchesToMeters(21.73);
        public static final double WHEEL_BASE = Units.inchesToMeters(21.73);
        public static final double WHEEL_DIAMETER = Units.inchesToMeters(4.0);
        public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
    
        public static final double DRIVE_GEAR_RATIO = (6.75 / 1.0); // 6.75:1
        public static final double ANGLE_GEAR_RATIO = ((150.0 / 7.0) / 1.0); // (150/7):1
    
        public static final SwerveDriveKinematics kSwerveKinematics =
            new SwerveDriveKinematics(
                new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
                new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0));
        
        /* Swerve Voltage Compensation */
        public static final double MAX_VOLTAGE = 12.0;
    
        /* Angle Motor PID Values */
        public static final double ANGLE_KP = 0.01;
        public static final double ANGLE_KI = 0.0;
        public static final double ANGLE_KD = 0.0;
        public static final double ANGLE_KFF = 0.0;
    
        /* Drive Motor PID Values */
        public static final double DRIVE_KP = 0.1;
        public static final double DRIVE_KI = 0.0;
        public static final double DRIVE_KD = 0.0;
        public static final double DRIVE_KFF = 0.0;
    
        /* Drive Motor Characterization Values */
        public static final double DRIVE_KS = 0.667;
        public static final double DRIVE_KV = 2.44;
        public static final double DRIVE_KA = 0.27;
    
        /* Drive Motor Conversion Factors */
        public static final double DRIVE_POSITION_CONVERSION_FACTOR =
            WHEEL_CIRCUMFERENCE / DRIVE_GEAR_RATIO;
        public static final double DRIVE_VELOCITY_CONVERSION_FACTOR = DRIVE_POSITION_CONVERSION_FACTOR / 60.0;
        public static final double ANGLE_POSITION_CONVERSION_FACTOR_DEGREES = 360.0 / ANGLE_GEAR_RATIO;
        public static final double ANGLE_POSITION_CONVERSION_FACTOR_RADIANS = Math.PI * 2;
        public static final double ANGLE_VELOCITY_CONVERSION_FACTOR_DEGREES = ANGLE_POSITION_CONVERSION_FACTOR_DEGREES / 60.0;
        public static final double ANGLE_VELOCITY_CONVERSION_FACTOR_RADIANS = ANGLE_POSITION_CONVERSION_FACTOR_RADIANS / 60.0;
        /* Swerve Profiling Values */
        public static final double MAX_VELOCITY = 4; // meters per second
        public static final double MAX_ANGULAR_VELOCITY = 9.42; //radians per second

        /* Motor Inverts */
        public static final boolean DRIVE_INVERT = false;
        public static final boolean ANGLE_INVERT = true;

        /* Angle Encoder Invert */
        public static final boolean CANCODER_INVERT = false;


        //Charge Station Board and Balance Speeds (-1 - 1)
        //public static final double CHARGE_STATION_BALANCE_SPEED = 0.3;
        //public static final double BOARD_CHARGE_SPEED = -0.55;

        /*Charge Station Board and Balance Speeds (meters/second)
        THESE SPEEDS WERE CALCULATED BASED ON THE CURRENT MAXIMUM VELOCITY
        CONSTANT AND ARE PLACEHOLDERS FOR NOW*/
        public static final double CHARGE_STATION_BALANCE_SPEED = 1.35;
        public static final double BOARD_CHARGE_SPEED = -2.475;

        // The minimum angle the drivetrain must be at to stop when boarding the charge station
        public static final double BOARD_CHARGE_MINIMUM_STOP_ANGLE = 6;
        // The amount the angle should drop below the maximum angle to stop boarding the charge station and begin balancing
        public static final double BOARD_CHARGE_ANGLE_CHANGE_THRESHOLD = 3.8;

        //Autnomous Tolerances
        public static final double AUTO_DISTANCE_ERROR_TOLERANCE = 0.25;

            
        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
          public static final int DRIVE_MOTOR_ID = 4;
          public static final int ANGLE_MOTOR_ID = 3;
          public static final int CANCODER_ID = 1;
          public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(327.48046875);
          public static final SwerveModuleConstants CONSTANTS =
              new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET);
        }

        /* Back Left Module - Module 1 */
        public static final class Mod1 {
          public static final int DRIVE_MOTOR_ID = 2;
          public static final int ANGLE_MOTOR_ID = 1;
          public static final int CANCODER_ID = 2;
          public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(55.01953125);
          public static final SwerveModuleConstants CONSTANTS =
              new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET);
          }
    
        /* Front Right Module - Module 2 */
        public static final class Mod2 {
          public static final int DRIVE_MOTOR_ID = 14;
          public static final int ANGLE_MOTOR_ID = 13;
          public static final int CANCODER_ID = 3;
          public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(286.34765625);
          public static final SwerveModuleConstants CONSTANTS =
              new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET);
        }

    
        /* Back Right Module - Module 3 */
        public static final class Mod3 {
          public static final int DRIVE_MOTOR_ID = 15;
          public static final int ANGLE_MOTOR_ID = 16;
          public static final int CANCODER_ID = 4;
          public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(67.939453125);
          public static final SwerveModuleConstants CONSTANTS =
              new SwerveModuleConstants(DRIVE_MOTOR_ID, ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET);
        }
      }
    
      public static final class kAutonomous {
        //THIS MAXIMUM LINEAR VELOCITY AND ACCELERATION ARE PLACEHOLDERS FOR NOW
        public static final double kMaxVelocityMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularVelocityRadiansPerSecond = Math.PI;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI;
    
        //THESE P GAINS ARE PLACEHOLDERS FOR NOW
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularVelocityRadiansPerSecond, kMaxAngularAccelerationRadiansPerSecondSquared);
      }

}