package frc.robot;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

public class Drivetrain {
  //Creates and Groups drivetrain motor objects
  private static final PWMSparkMax leftMotorFront = new PWMSparkMax(3);
  private static final PWMSparkMax leftMotorBack = new PWMSparkMax(4);
  private static final PWMSparkMax rightMotorFront = new PWMSparkMax(1);
  private static final PWMSparkMax rightMotorBack = new PWMSparkMax(2);
  private static final MotorControllerGroup leftMotors = new MotorControllerGroup(leftMotorFront, leftMotorBack);
  private static final MotorControllerGroup rightMotors = new MotorControllerGroup(rightMotorFront, rightMotorBack);
  private static final DifferentialDrive robotDrivetrain = new DifferentialDrive(leftMotors, rightMotors);

  public static void drivetrainInit() {
    // Inverts the right Drivetrain motors
    rightMotors.setInverted(true);
  }

  public static void runDrivetrain() {

    // Drive with arcade drive. Left Y axis drives forward/backward, and Right X axis turns.
    robotDrivetrain.arcadeDrive(-Robot.leftJoystick.getY(), Robot.rightJoystick.getX());

  }
}
