// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;
//import com.ctre.phoenix.motorcontrol.can.TalonFX;
import frc.robot.RobotContainer;
//import frc.robot.subsystems.Intake;

public class IntakeCommand extends CommandBase {

  private final Joystick leftJoystick;
  private final Joystick rightJoystick;

  boolean intakeForwardToggle = false;

  public IntakeCommand(Intake intake, Joystick leftJoystick, Joystick rightJoystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_intake);

    this.leftJoystick = leftJoystick;
    this.rightJoystick = rightJoystick;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    boolean intakeForwardPressed = rightJoystick.getRawButtonPressed(2);
    boolean intakeBackward = leftJoystick.getRawButton(2);

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
  public void end(boolean interrupted) {

    RobotContainer.m_intake.runIntake(false, false);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return false;
    
  }
}