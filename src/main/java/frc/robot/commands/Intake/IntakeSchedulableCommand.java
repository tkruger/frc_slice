// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;
//import com.ctre.phoenix.motorcontrol.can.TalonFX;
import frc.robot.RobotContainer;
//import frc.robot.subsystems.Intake;

public class IntakeSchedulableCommand extends CommandBase {

  boolean intakeForwardPressed = false;
  boolean intakeForwardToggle = false;
  boolean intakeBackward = false;

  private int direction;

  public IntakeSchedulableCommand(Intake intake, int direction) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_intake);
    
    this.direction = direction;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(direction == 1){
      intakeForwardPressed = true;
      intakeBackward = false;
    } else if(direction == -1){
      intakeBackward = true;
      intakeForwardPressed = false;
    } else{
      intakeForwardPressed = false;
      intakeBackward = false;
    }

    if(intakeForwardPressed == true) {
      if(intakeForwardToggle == true) {
      intakeForwardToggle = false;
      }
      else if (intakeForwardToggle == false) {
      intakeForwardToggle = true;
      }
    }

    RobotContainer.m_intake.runIntake(intakeForwardToggle, intakeBackward);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(intakeForwardPressed == true){
      return true;
    } else{
      return false;
    }
  }
}
