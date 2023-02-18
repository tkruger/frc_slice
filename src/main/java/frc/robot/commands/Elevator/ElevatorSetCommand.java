// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

public class ElevatorSetCommand extends CommandBase {

  private final Elevator m_elevator;
  private final double m_targetPosition;

  private boolean leftTargetPositionReached;
  private boolean rightTargetPositionReached;

  /** Creates a new ElevatorSetCommand. */
  public ElevatorSetCommand(Elevator elevator, double targetPosition) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);

    m_elevator = elevator;
    m_targetPosition = targetPosition;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    leftTargetPositionReached = false;
    rightTargetPositionReached = false;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(m_targetPosition < m_elevator.getLeftMotorPosition() - 1) {

      m_elevator.runLeftMotor(true, Constants.elevator_SET_SPEED);

    }
    else if(m_targetPosition > m_elevator.getLeftMotorPosition() + 1){

      m_elevator.runLeftMotor(false, Constants.elevator_SET_SPEED);

    }
    else {

      leftTargetPositionReached = true;

    }

    if(m_targetPosition < m_elevator.getRightMotorPosition() - 1) {

      m_elevator.runRightMotor(true, Constants.elevator_SET_SPEED);

    }
    else if(m_targetPosition > m_elevator.getRightMotorPosition() + 1) {

      m_elevator.runRightMotor(false, Constants.elevator_SET_SPEED);

    }
    else {

      rightTargetPositionReached = true;

    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return (leftTargetPositionReached && rightTargetPositionReached);
    
  }
  
}
