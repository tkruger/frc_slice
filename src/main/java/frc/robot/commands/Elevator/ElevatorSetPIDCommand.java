// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class ElevatorSetPIDCommand extends CommandBase {

  private final Elevator m_elevator;
  private final double m_targetPosition;

  /** Creates a new ElevatorSetCommand. */
  public ElevatorSetPIDCommand(Elevator elevator, double targetPosition) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);

    m_elevator = elevator;
    m_targetPosition = targetPosition;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    //m_elevator.setPID(Constants.Elevator.KP, Constants.Elevator.KI, Constants.Elevator.KD);
    m_elevator.setPosition(m_targetPosition);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevator.runElevator(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return m_elevator.atTargetPosition();
    
  }
  
}
