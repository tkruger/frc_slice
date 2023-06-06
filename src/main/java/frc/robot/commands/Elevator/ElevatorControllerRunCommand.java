// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.robot.JoystickFilter;
import frc.robot.subsystems.Elevator;

public class ElevatorControllerRunCommand extends CommandBase {

  private final Elevator m_elevator;

  private final CommandGenericHID m_manipulatorController;

  private final JoystickFilter speedFilter;

  /** Creates a new ElevatorJoystickRunCommand. */
  public ElevatorControllerRunCommand(Elevator elevator, CommandGenericHID manipulatorController) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);

    m_elevator = elevator;
    m_manipulatorController = manipulatorController;

    speedFilter = new JoystickFilter(0.05, 0.3, false);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double elevatorSpeed = speedFilter.filter(m_manipulatorController.getRawAxis(1));

    m_elevator.runElevator(elevatorSpeed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_elevator.runElevator(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return false;

  }

}
