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

  private double elevatorSpeed;

  private double leftSpeed;
  private double rightSpeed;

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

    elevatorSpeed = Constants.elevator_SET_SPEED;

    leftSpeed = 0;
    rightSpeed = 0;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(m_targetPosition + 1 < m_elevator.getLeftMotorPosition()) {

      leftSpeed = -elevatorSpeed;

    }
    else if(m_targetPosition - 1 > m_elevator.getLeftMotorPosition()){

      leftSpeed = elevatorSpeed;

    }
    else {

      leftTargetPositionReached = true;
      leftSpeed = 0;

    }

    if(m_targetPosition + 1 < m_elevator.getRightMotorPosition()) {

      rightSpeed = -elevatorSpeed;

    }
    else if(m_targetPosition - 1 > m_elevator.getRightMotorPosition()) {

      rightSpeed = elevatorSpeed;

    }
    else {

      rightTargetPositionReached = true;
      rightSpeed = 0;

    }

    m_elevator.runLeftMotor(leftSpeed);
    m_elevator.runRightMotor(rightSpeed);

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
