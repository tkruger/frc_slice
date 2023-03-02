// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

public class CalibrateElevatorCommand extends CommandBase {
  private final Elevator m_elevator;
  /** Creates a new ElevatorLimitSwitchResetCommand. */
  public CalibrateElevatorCommand(Elevator elevator) {
    m_elevator = elevator;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_elevator.runElevator(Constants.Elevator.CALIBRATION_SPEEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevator.runElevator(0);
    m_elevator.setPosition(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_elevator.atBottom();
  }
}
