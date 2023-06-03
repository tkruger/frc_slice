// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LEDs;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.LEDs.LEDMode;

public class VariableModeLEDs extends CommandBase {

  private final LEDs m_leds;

  private final RainbowLEDs m_rainbowLEDs;
  private final CustomRainbowLEDs m_solidOrangeLEDs;
  private final FlashColorCommand m_flashOrangeLEDs;

  private LEDMode ledMode;
  private LEDMode lastLEDMode;

  private Command ledCommand;

  /** Creates a new VariableModeLEDs. */
  public VariableModeLEDs(
    LEDs leds,
    RainbowLEDs rainbowLEDs, 
    CustomRainbowLEDs solidOrangeLEDs, 
    FlashColorCommand flashOrangeLEDs) {

    m_leds = leds;

    m_rainbowLEDs = rainbowLEDs;
    m_solidOrangeLEDs = solidOrangeLEDs;
    m_flashOrangeLEDs = flashOrangeLEDs;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(leds);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    ledMode = m_leds.getSelectedMode();

    if(ledMode != lastLEDMode) {
    
      switch (ledMode) {

        case SOLID_ORANGE:
          ledCommand = m_solidOrangeLEDs;
          break;
        case FLASHING_ORANGE:
          ledCommand = m_flashOrangeLEDs;
          break;
        default:
          ledCommand = m_rainbowLEDs;
          break;
  
      }

      ledCommand.schedule();

    }

    lastLEDMode = m_leds.getSelectedMode();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    ledCommand.cancel();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

}
