// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
//import com.ctre.phoenix.motorcontrol.can.TalonFX;
import frc.robot.RobotContainer;
//import frc.robot.subsystems.Intake;

public class IntakeRun extends CommandBase {
  /** Creates a new RunIntake. */

  public IntakeRun() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_intake);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    boolean intakeForwardPressed = RobotContainer.leftJoystick.getRawButtonPressed(2);
    boolean intakeBackwardPressed = RobotContainer.leftJoystick.getRawButtonPressed(3);

    boolean intakeMotorForwardToggle = false;
    boolean intakeMotorBackwardToggle = false;

    if(intakeForwardPressed == true) {
      if(intakeMotorForwardToggle == true) {
      intakeMotorForwardToggle = false;
      }
      else if (intakeMotorForwardToggle == false) {
      intakeMotorForwardToggle = true;
      }
    }

    if(intakeBackwardPressed == true) {
      if(intakeMotorBackwardToggle == true) {
          intakeMotorBackwardToggle = false;
      }
      else if(intakeMotorBackwardToggle == false) {
          intakeMotorBackwardToggle = true;
      }
    }

    RobotContainer.m_intake.runIntake(intakeMotorForwardToggle, intakeMotorBackwardToggle);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    RobotContainer.m_intake.runIntake(false, false);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return false;
    
  }
}
