// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Schedules the given command when this command is initialized and cancels it when ended. 
 *
 * <p>This class is a custom version of the WPILib ScheduleCommand.
 */
public class LambdaCommand extends CommandBase {
  private final Supplier<Command> m_toScheduleSupplier;
  private Command m_finalToSchedule;

  /**
   * Creates a new LambdaCommand that schedules the current value of the given command
   * supplier when initialized and cancels it when ended.
   *
   * @param toScheduleSupplier the supplier of the command to schedule
   */
  public LambdaCommand(Supplier<Command> toScheduleSupplier) {
    m_toScheduleSupplier = toScheduleSupplier;
  }

  @Override
  public void initialize() {
    m_finalToSchedule = m_toScheduleSupplier.get();
    m_finalToSchedule.schedule();
  }

  @Override
  public void end(boolean interrupted) {
    m_finalToSchedule.cancel();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
