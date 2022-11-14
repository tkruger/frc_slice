/// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.shuffleboard.*;

import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.kauailabs.navx.frc.AHRS;

import com.swervedrivespecialties.swervelib.*;

public class SwerveDrivetrain extends SubsystemBase {
  
  //Creates drivetrain motor objects and groups
  private final SwerveModule leftModuleFront, leftModuleBack, rightModuleFront, rightModuleBack;
  private final SwerveDriveKinematics m_swerveKinematics;

  private final AHRS navXGyro;

  public double leftFrontLastPosition, leftBackLastPosition, rightFrontLastPosition, rightBackLastPosition;

  // The current target position of every motor
  public double leftTargetPositionFront, leftTargetPositionBack, rightTargetPositionFront, rightTargetPositionBack;

  /** Creates a new Drivetrain. */
  public SwerveDrivetrain() {
    //The gear ratios for these object declarations are placholders for now
    leftModuleFront = Mk4iSwerveModuleHelper.createNeo(
      Shuffleboard.getTab("SmartDashboard").getLayout("Left Front Module", BuiltInLayouts.kList), 
      Mk4iSwerveModuleHelper.GearRatio.L1, 
      0, 
      0, 
      0, 
      0);

    leftModuleBack = Mk4iSwerveModuleHelper.createNeo(
      Shuffleboard.getTab("SmartDashboard").getLayout("Left Back Module", BuiltInLayouts.kList), 
      Mk4iSwerveModuleHelper.GearRatio.L1, 
      0, 
      0, 
      0, 
      0);

    rightModuleFront = Mk4iSwerveModuleHelper.createNeo(
      Shuffleboard.getTab("SmartDashboard").getLayout("Right Front Module", BuiltInLayouts.kList), 
      Mk4iSwerveModuleHelper.GearRatio.L1, 
      0, 
      0, 
      0, 
      0);

    rightModuleBack = Mk4iSwerveModuleHelper.createNeo(
      Shuffleboard.getTab("SmartDashboard").getLayout("Right Back Module", BuiltInLayouts.kList), 
      Mk4iSwerveModuleHelper.GearRatio.L1, 
      0, 
      0, 
      0, 
      0);

      //This kinematics object uses the total length of the robot(0.8128 meters) for now as a placholder for the wheelbase length
      m_swerveKinematics = new SwerveDriveKinematics(
          // Front left
          new Translation2d(Constants.kTrackWidthMeters / 2.0, 0.8128 / 2.0),
          // Front right
          new Translation2d(Constants.kTrackWidthMeters / 2.0, -0.8128 / 2.0),
          // Back left
          new Translation2d(-Constants.kTrackWidthMeters  / 2.0, 0.8128 / 2.0),
          // Back right
          new Translation2d(-Constants.kTrackWidthMeters  / 2.0, -0.8128 / 2.0)
      );

    navXGyro = new AHRS(SerialPort.Port.kUSB1);

    // Display current gyro heading on Shuffleboard
    Shuffleboard.getTab("SmartDashboard").add(navXGyro);    
  }

  @Override
  public void periodic() {
    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void swerveDrive(ChassisSpeeds chassisSpeeds) {

    SwerveModuleState[] states = m_swerveKinematics.toSwerveModuleStates(chassisSpeeds);
    //SwerveDriveKinematics.normalizeWheelSpeeds(states, Constants.kMaxSpeedMetersPerSeconds);

    //The maximum voltage value of 10 is taken from Trajectories
    leftModuleFront.set(states[0].speedMetersPerSecond / Constants.kMaxSpeedMetersPerSeconds * 10, states[0].angle.getRadians());
    leftModuleBack.set(states[1].speedMetersPerSecond / Constants.kMaxSpeedMetersPerSeconds * 10, states[1].angle.getRadians());
    rightModuleFront.set(states[2].speedMetersPerSecond / Constants.kMaxSpeedMetersPerSeconds * 10, states[2].angle.getRadians());
    rightModuleBack.set(states[2].speedMetersPerSecond / Constants.kMaxSpeedMetersPerSeconds * 10, states[2].angle.getRadians());

  }

  public static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  public double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.05);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
  
  public double getHeading() {

    return -navXGyro.getYaw() + 180;

  }

  public double getTurnRate() {

    return navXGyro.getRate();

  }

}
