// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
THIS COMMAND IS DEPRECATED, USE ALIGNLESS OR ALIGNED SHOOT SEQUENCE INSTEAD
*/

package frc.robot.commands.Shooter;

import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** A Shooter command that uses a shooter subsystem. */
public class ShooterCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Shooter m_shooter;

  double primarySpeed = 0;
  double secondarySpeed = 0;

  private final Joystick leftJoystick;
  private final Joystick rightJoystick;

  /**
   * @param subsystem The subsystem used by this command.
   */
  public ShooterCommand(Shooter shooter, Joystick leftJoystick, Joystick rightJoystick) {
    m_shooter = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);

    this.leftJoystick = leftJoystick;
    this.rightJoystick = rightJoystick;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //Sets Shooter flywheels
    if(leftJoystick.getRawButton(11)) {
      primarySpeed = rightJoystick.getZ();
      secondarySpeed = leftJoystick.getZ();

      //PWM only (too low for PID)
      //primarySpeed = -((primarySpeed + 1) / 2);
      //secondarySpeed = (secondarySpeed + 1) / 2;

      //PID ONLY (way too high for PWM)
      primarySpeed = -((primarySpeed + 1) * 710);
      secondarySpeed = (secondarySpeed + 1) * 710;

    }
    else {
      primarySpeed = 0;
      secondarySpeed = 0;
    }

    System.out.println("ShooterCommand");

    m_shooter.SetShooters(primarySpeed, secondarySpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.SetShooters(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
