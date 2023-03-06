// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.factories.SparkMaxFactory;

public class Intake extends SubsystemBase {
 
  private final CANSparkMax mandibleMotor, rotateMotor;

  private final RelativeEncoder mandibleEncoder, rotateEncoder;

  private final SparkMaxPIDController mandiblePID/*, rotatePID*/;

  private boolean mandibleClosed;

  private final ShuffleboardTab manipulatorTab;

  private final SimpleWidget mandibleClosedWidget, mandiblePositionWidget;

  /** Creates a new Elevator. */
  public Intake() {

    mandibleMotor = SparkMaxFactory.createDefaultSparkMax(Constants.Intake.SPIN_PORT);
    rotateMotor = SparkMaxFactory.createDefaultSparkMax(Constants.Intake.MANDIBLE_PORT);

    mandibleEncoder = mandibleMotor.getEncoder(Type.kHallSensor, Constants.Encoder.CPR);
    rotateEncoder = rotateMotor.getEncoder(Type.kHallSensor, Constants.Encoder.CPR);

    mandiblePID = mandibleMotor.getPIDController();
    //rotatePID = rotateMotor.getPIDController();

    mandibleClosed = false;

    manipulatorTab = Shuffleboard.getTab("Manipulator Tab");

    mandibleClosedWidget = manipulatorTab.add("Mandibles Closed", false).withPosition(4, 1).withSize(1, 1);
    mandiblePositionWidget = manipulatorTab.add("Mandibles Position", 0).withPosition(3, 0).withSize(3, 1);

  }

  public void setMandiblePID(double kP, double kI, double kD) {
    mandiblePID.setP(kP);
    mandiblePID.setI(kI);
    mandiblePID.setD(kD);
  } 

  /**
   * Opens the mandibles
   */
  public void openMandibles() {
    mandiblePID.setReference(Constants.Intake.MANDIBLE_OPEN_POSITION, ControlType.kPosition);
    mandibleClosed = false;
  }

  /**
   * Close the mandibles
   */
  public void closeMandibles() {
    mandiblePID.setReference(Constants.Intake.MANDIBLE_CLOSED_POSITION, ControlType.kPosition);
    mandibleClosed = true;
  }

  /**
   * Manually spin the mandible motor at a given speed
   * @param speed the speed the mandible motor spins at, from 1 to -1
   */
  public void runMandibles(double speed) {
    mandibleMotor.set(speed);
  }

  /**
   * Spins the wheels along the intake
   * @param speed the speed the wheels spin at, from 1 to -1. If positive, intake. if negative, eject.
   */
  public void runIntake(double speed) {
    // SKELETON CODE NOTE: might need to reverse speed so positive is intake and negative is eject
    rotateMotor.set(speed);
  }

  /**
   * @return the angular position of the mandibles, in [UNITS GO HERE]
   */
  public double getMandibleEncoderPosition() {

    return mandibleEncoder.getPosition();

  }

  /**
   * Resets the position reported by the encoder of the mandibles to the desired position
   * @param position the position the encoder will be set to, in rotations
   */
  public void setMandibleEncoderPosition(double position) {
    mandibleEncoder.setPosition(position);
  }

  /**
   * @return the angular position of the intake wheels, in [UNITS GO HERE]
   */
  public double getRotateEncoderPosition() {

    return rotateEncoder.getPosition();

  }

  public boolean mandibleVoltageSpike() {
    return mandibleMotor.getOutputCurrent() > Constants.Intake.CALIBRATION_CURRENT_THRESHOLD;
  }

  /**
   * 
   * @return true if the mandibles are closed, and false if the mandibles are open
   */
  public boolean getMandibleClosed() {
    return mandibleClosed;
  }

  public void callibratePreparation() {
    mandibleClosed = false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    mandibleClosedWidget.getEntry().setBoolean(getMandibleClosed());
    mandiblePositionWidget.getEntry().setDouble(getMandibleEncoderPosition());
  }

}
