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

    //Define Joystick Constants
    public final class Joysticks {

        //Define Joystick Ports
        public static final int LEFT_JOYSTICK_PORT = 0;
        public static final int RIGHT_JOYSTICK_PORT = 1;
        //(This is a placeholder for now)
        public static final int MANIPULATOR_JOYSTICK_PORT = 2;

        public static final double A_COEFFICIENT = 0.7;
        public static final double B_COEFFICIENT = 0.3;
        public static final int FIRST_POWER = 3;
        public static final int SECOND_POWER = 1;

    }

    //Define Drivetrain Constants
    public final static class Drivetrain {

        //Define Motor Ports
        public static final int LEFT_FRONT_PORT = 3;
        public static final int LEFT_BACK_PORT = 4;
        public static final int RIGHT_FRONT_PORT = 7;
        public static final int RIGHT_BACK_PORT = 8;

        //Define Encoder Constants
        public static final double WHEEL_DIAMETER_METERS = 0.15;

        public static final double VELOCITY_CONVERSION_FACTOR = Math.PI * WHEEL_DIAMETER_METERS /(60 * 10.75);
        public static final double POSITION_CONVERSION_FACTOR = Math.PI * WHEEL_DIAMETER_METERS / 10.75;

        //Define Drivetrain Kinematics Object
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(0.5842);

        //Define Charge Station Balance Constants
        public static final double CHARGE_STATION_BALANCE_SPEED = 0.3;

        //Autnomous Tolerances
        public static final double AUTO_DISTANCE_ERROR_TOLERANCE = 0.25;

    }

    //Define Encoder Constants
    public final class Encoder {

        public static final int CPR = 42;

    }

    //Define Elevator Constants
    public final class Elevator {

        //Define Motor Ports (these are placeholders for now)
        public static final int LEFT_PORT = 1;
        public static final int RIGHT_PORT = 2;

        //P, I, and D gains for the elevator when going to a set position (these are placeholders)
        public static final double KP = 0.1;
        public static final double KI = 0;
        public static final double KD = 0;

        //Define Elevator Limit Switch Channels (this is a placeholder for now) 
        public static final int LIMIT_SWITCH_CHANNEL = 9;

        //Define Motor Speeds (these are placholders for now)
        public static final double RUN_SPEED = 0.5;
        public static final double SET_SPEED = 0.7; // May be deprecated with PID
        public static final double CALIBRATION_SPEEED = 0.3;

        public static final double POSITIONAL_ERROR_THRESHOLD = 4;
        public static final double POSITIONAL_MAX_SPEED = 0.6;
    }

    //Define Wrist Constants
    public final class Wrist {

        //Define Motor Ports
        public static final int MOTOR_PORT = 12;

        //Define Limit Switch Channels (this is a placeholder for now)
        public static final int LIMIT_SWITCH_CHANNEL = 30;

        //Define angles the intake is at from horizontal at maximum and minimum points (this is a placeholder for now)
        public static final double MAX_ANGLE = 105;
        public static final double MIN_ANGLE = -105;

        //Define how fast the wrist spins when resetting angle using limit switch (this is a placeholder for now)
        public static final double RESET_SPEED = 0.3;

        //P, I, and D gains for the wrist when going to a set position
        public static final double KP = 0.018;
        public static final double KI = 0.000037;
        public static final double KD = 0.0016;

        public static final double RUN_UP_SPEED = 0.3;
        public static final double RUN_DOWN_SPEED = 0.2;

        public static final double POSITION_CONVERSION_FACTOR = (18 * 360) / 1500;

        public static final double CALIBRATE_CURRENT_THRESHOLD = 40;

        public static final double POSITIONAL_ERROR_THRESHOLD = 6;
        public static final double POSITIONAL_MAX_SPEED = 0.35;

    }

    //Define Intake Constants
    public final class Intake {

        //Define Motor Ports
        public static final int SPIN_PORT = 5;
        public static final int MANDIBLE_PORT = 9;

        //Define open and closed positions
        public static final double MANDIBLE_CLOSED_POSITION = 0;
        public static final double MANDIBLE_OPEN_POSITION = 0;

        //P, I, and D gains for the mandibles when going to a set position
        public static final double MANDIBLE_KP = 0.1;
        public static final double MANDIBLE_KI = 0;
        public static final double MANDIBLE_KD = 0;

        //Define threshold current (in amps) we must exceed so we know the intake is open/closed (this is a placeholder)
        public static final double CALIBRATION_CURRENT_THRESHOLD = 7.0;

        //Define the speed the madibles run at when manually running
        public static final double MANDIBLE_RUN_SPEED = 0.2;
        //Define the speed the mandibles run at when calibrating (this is a placeholder)
        public static final double CALIBRATION_SPEED = -0.1;
        
    }

    //Define Limelight Constants
    public final class Limelight {

        //Define Alignment Speeds (these are placeholders for now)
        public static final double TURN_ALIGNMENT_SPEED = 0.5;
        public static final double FORWARD_ALIGNMENT_SPEED = 0.4;

    }

    public final class LEDs {
        public static final int port = 9;
        public static final int count = 77;
    }

    //Define RobotState Constants
    public static final class States {

        public static final RobotState LOW_ROW_GROUND_STATE = new RobotState(1, -5);
        public static final RobotState MID_ROW_CUBE_STATE = new RobotState(38, -50);
        public static final RobotState MID_ROW_CONE_STATE = new RobotState(48, -50);
        public static final RobotState HIGH_ROW_CUBE_STATE = new RobotState(98, -25);
        public static final RobotState HIGH_ROW_CONE_STATE = new RobotState(98, -40);
        public static final RobotState DOUBLE_SUBSTATION_STATE = new RobotState(85, -27);
        public static final RobotState TRAVEL_STATE = new RobotState(1, -90);

        public static final RobotState TRANSITION_OUT_STATE = new RobotState(1, -60);
        public static final RobotState TRANSITION_HIGH_STATE = new RobotState(88, -60);
        public static final RobotState TRANSITION_MID_STATE = new RobotState(38, -60);
        public static final RobotState TRANSITION_DOUBLE_SUBSTATION_STATE = new RobotState(DOUBLE_SUBSTATION_STATE.elevatorHeight, -60);

    }

    //Define Autonomous Constants
    public final class Autonomous {

        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;

        public static final double ksVolts = 0.2492;
        public static final double kvVoltsSecondsPerMeter = 2.8532;
        public static final double kaVoltsSecondsSquaredPerMeter = 0.57535;
    
        public static final double kPDriveVel = 0.00029423;

    }

}
