package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import frc.robot.Robot;

public class Drivetrain {
  //Creates and Groups drivetrain motor objects
  private static final PWMSparkMax leftMotorFront = new PWMSparkMax(3);
  private static final PWMSparkMax leftMotorBack = new PWMSparkMax(4);
  private static final PWMSparkMax rightMotorFront = new PWMSparkMax(1);
  private static final PWMSparkMax rightMotorBack = new PWMSparkMax(2);
  private static final MotorControllerGroup leftMotors = new MotorControllerGroup(leftMotorFront, leftMotorBack);
  private static final MotorControllerGroup rightMotors = new MotorControllerGroup(rightMotorFront, rightMotorBack);
  private static final DifferentialDrive robotDrivetrain = new DifferentialDrive(leftMotors, rightMotors);

  //Declare Quickturn Variables
  static double startAngle;
  static double endAngle;
  static double forwardSpeed;
  static double turnSpeed;

  public static void drivetrainInit() {
    // Inverts the right Drivetrain motors
    rightMotors.setInverted(true);
  }

  public static void runDrivetrain() {

    //Initiate quickTurn function
    //Currently prints gyro angle
    if(Robot.leftJoystick.getRawButton(7) == true) {
      startAngle = GyroCode.robotAngle;
      endAngle = startAngle + 180;

      System.out.println(startAngle);
      quickTurn();
    }

    //Sets robot speed and turn speed
    double forwardSpeed = -Robot.leftJoystick.getY();
    double turnSpeed = Robot.rightJoystick.getX();

    // Drive with arcade drive. Left Y axis drives forward/backward, and Right X axis turns.
    robotDrivetrain.arcadeDrive(forwardSpeed, turnSpeed);

  }

  //Turns robot 180 degrees
  public static void quickTurn() {
    forwardSpeed = 0;
    turnSpeed = 0.5;
    int iterations = 0;

    while(iterations <= 20) {
      robotDrivetrain.arcadeDrive(forwardSpeed, turnSpeed);
    }
    iterations = 0;

    if(GyroCode.robotAngle <= startAngle) {
      turnSpeed = -turnSpeed;
    }

    while(GyroCode.robotAngle <= endAngle) {
      robotDrivetrain.arcadeDrive(forwardSpeed, turnSpeed);
    }
  }

}
