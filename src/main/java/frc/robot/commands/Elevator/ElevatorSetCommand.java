// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;

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

    if(m_targetPosition < m_elevator.getLeftEncoderPosition() - 1) {

      m_elevator.runLeftMotor(true, 0.5);

    }
    else if(m_targetPosition > m_elevator.getLeftEncoderPosition() + 1){

      m_elevator.runLeftMotor(false, 0.5);

    }
    else {

      leftTargetPositionReached = true;

    }

    if(m_targetPosition < m_elevator.getRightEncoderPosition() - 1) {

      m_elevator.runRightMotor(true, 0.5);

    }
    else if(m_targetPosition > m_elevator.getRightEncoderPosition() + 1) {

      m_elevator.runRightMotor(false, 0.5);

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
