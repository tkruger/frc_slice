package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import com.revrobotics.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Robot;

public class Drivetrain {
  //Creates and Groups drivetrain motor objects
  private final CANSparkMax leftMotorFront = new CANSparkMax(3, MotorType.kBrushless);
  private final CANSparkMax leftMotorBack = new CANSparkMax(4, MotorType.kBrushless);
  private final CANSparkMax rightMotorFront = new CANSparkMax(1, MotorType.kBrushless);
  private final CANSparkMax rightMotorBack = new CANSparkMax(2, MotorType.kBrushless);
  private final MotorControllerGroup leftMotors = new MotorControllerGroup(leftMotorFront, leftMotorBack);
  private final MotorControllerGroup rightMotors = new MotorControllerGroup(rightMotorFront, rightMotorBack);
  private final DifferentialDrive robotDrivetrain = new DifferentialDrive(leftMotors, rightMotors);

  //Declare Quickturn Variables
  double startAngle;
  double endAngle;
  double forwardSpeed;
  double turnSpeed;

  //Instantiate Classes
  private GyroCode robotAngle = new GyroCode();
  private Robot joysticks = new Robot();

  public void drivetrainInit() {
    // Inverts the right Drivetrain motors
    rightMotors.setInverted(true);
  }

  public void runDrivetrain() {

    //Initiate quickTurn function
    //Currently prints gyro angle
    if(joysticks.leftJoystick.getRawButton(7) == true) {
      startAngle = robotAngle.gyroUpdate();
      endAngle = startAngle + 180;

      System.out.println(startAngle);
      quickTurn();
    }

    //Sets robot speed and turn speed
    double forwardSpeed = -joysticks.leftJoystick.getY();
    double turnSpeed = joysticks.rightJoystick.getX();

    // Drive with arcade drive. Left Y axis drives forward/backward, and Right X axis turns.
    robotDrivetrain.arcadeDrive(forwardSpeed, turnSpeed);

  }

  //Turns robot 180 degrees
  public void quickTurn() {
    forwardSpeed = 0;
    turnSpeed = 0.5;
    int iterations = 0;

    while(iterations <= 20) {
      robotDrivetrain.arcadeDrive(forwardSpeed, turnSpeed);
    }
    iterations = 0;

    if(robotAngle.gyroUpdate() <= startAngle) {
      turnSpeed = -turnSpeed;
    }

    while(robotAngle.gyroUpdate() <= endAngle) {
      robotDrivetrain.arcadeDrive(forwardSpeed, turnSpeed);
    }
  }

}
