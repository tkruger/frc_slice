// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Pneumatics;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Button;
import frc.robot.subsystems.Pneumatics;


/** Creates a new PneumaticsIdleCommand. */
public class PneumaticsIdleCommand extends CommandBase {
  
  private final Pneumatics m_Pneumatics;

  /* 
   * don't know if these variables will be used
  private static Joystick leftJoystick = Button.leftJoystick;
  private static Joystick rightJoystick = Button.rightJoystick;
  private static JoystickButton onButton = Button.leftButton1;
  private static JoystickButton OffButton = Button.leftButton2;
  */


  public PneumaticsIdleCommand(Pneumatics m_pneumatics) {
    m_Pneumatics = m_pneumatics;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Pneumatics);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
