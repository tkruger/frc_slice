// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import frc.robot.RobotContainer;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ShooterCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Shooter m_shooter;

  double primarySpeed = 0;
  double secondarySpeed = 0;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ShooterCommand(Shooter shooter) {
    m_shooter = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //Sets Shooter flywheels
    if(RobotContainer.leftJoystick.getRawButton(1)) {
      primarySpeed = RobotContainer.rightJoystick.getZ();
      secondarySpeed = RobotContainer.leftJoystick.getZ();

      primarySpeed = -((primarySpeed + 1) / 2);
      secondarySpeed = (secondarySpeed + 1) / 2;
    }
    else {
      primarySpeed = 0;
      secondarySpeed = 0;
    }

    RobotContainer.m_shooter.SetShooters(primarySpeed, secondarySpeed);
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
