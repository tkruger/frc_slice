// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

public class ElevatorRunRightCommand extends CommandBase {

  private final Elevator m_elevator;
  private final boolean m_runUpwards;

/** Creates a new ElevatorRunCommand. */
  public ElevatorRunRightCommand(Elevator elevator, boolean runUpwards) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);

    m_elevator = elevator;
    m_runUpwards = runUpwards;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(m_runUpwards) {

      m_elevator.runRightMotor(Constants.Elevator.RUN_SPEED);

    }
    else {

      m_elevator.runRightMotor(-Constants.Elevator.RUN_SPEED);

    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_elevator.runRightMotor(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return false;
    
  }
  
}