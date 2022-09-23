// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.*;
//import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.*;
//import com.revrobotics.CANSparkMax.ControlType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter extends SubsystemBase {
  
  //Creates shooter motor, pid controllers, and encoder objects
  private final CANSparkMax primaryFlywheel, secondaryFlywheel;
  private final SparkMaxPIDController primaryPidController, secondaryPidController;
  private final RelativeEncoder primaryEncoder, secondaryEncoder;

  /** Creates a new Shooter. */
  public Shooter() {
    
    //Instantiates motors
    primaryFlywheel = new CANSparkMax(Constants.shooter_FLYWHEEL_PRIMARY_PORT, MotorType.kBrushless);
    secondaryFlywheel = new CANSparkMax(Constants.shooter_FLYWHEEL_SECONDARY_PORT, MotorType.kBrushless);

    //Instantiates pid controllers
    primaryPidController = primaryFlywheel.getPIDController();
    secondaryPidController = secondaryFlywheel.getPIDController();

    //Sets pid controller P, I, and D values
    primaryPidController.setP(0.10269);
    primaryPidController.setI(0);
    primaryPidController.setD(0);

    secondaryPidController.setP(0.1692);
    secondaryPidController.setI(0);
    secondaryPidController.setD(0);

    //Instantiates motor encoders
    primaryEncoder = primaryFlywheel.getEncoder();
    secondaryEncoder = secondaryFlywheel.getEncoder();

    primaryEncoder.setVelocityConversionFactor(2);
    secondaryEncoder.setVelocityConversionFactor(2);

    primaryFlywheel.restoreFactoryDefaults();
    secondaryFlywheel.restoreFactoryDefaults();

    secondaryFlywheel.setIdleMode(CANSparkMax.IdleMode.kCoast);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void SetShooters(double primarySpeed, double secondarySpeed) { 

    //set motor speed with PID (experimental)
    //primaryPidController.setReference(primarySpeed, CANSparkMax.ControlType.kVelocity);
    //secondaryPidController.setReference(secondarySpeed, CANSparkMax.ControlType.kVelocity);
    
    //set motor speed with PWM (deprecated)
    primaryFlywheel.set(primarySpeed);
    secondaryFlywheel.set(secondarySpeed);

    SmartDashboard.putNumber("Lower Shooter", primarySpeed);
    SmartDashboard.putNumber("Upper Shooter", secondarySpeed);  
    
  }

  // Shooter Math
  // https://www.desmos.com/calculator/jyxaoog9fd
  public double GetShotAngle(double distance) {
    double x2 = distance;
    double y2 = Constants.hubHeight;
    double x1 = x2 + Constants.secondaryPointDistance;
    double y1 = y2 + Constants.secondaryPointHeight;

    //double A = ((x2*y1) - (x1 * y2))/((Math.pow(x1, 1) * x2)-(Math.pow(x2, 2) * x1));
    double B = ((-Math.pow(x2, 2) * y1)+(Math.pow(x1, 2) * y2))/((Math.pow(x1, 2) * x2)-(Math.pow(x2, 2) * x1));
    double shotAngle = Math.toDegrees(Math.atan(B));
    return shotAngle;
  }

  public double getDistance(double angleToGoal) {
    double trueAngle = angleToGoal + Constants.limelightAngle;
    double height = Constants.hubHeight - Constants.limelightHeight; 
    double distance = height / Math.tan(Math.toRadians(trueAngle));
    distance += Constants.goalRadius;
    return distance;
  }
}
