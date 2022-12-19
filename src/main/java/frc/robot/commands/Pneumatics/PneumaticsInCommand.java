// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Pneumatics;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pneumatics;

public class PneumaticsInCommand extends CommandBase {
  //Declaring Variables
  private final Pneumatics m_Pneumatics;


  /** Creates a new PneumaticsInCommand. */
  public PneumaticsInCommand(Pneumatics m_pneumatics) {
    m_Pneumatics = m_pneumatics;
    // Use addRequirements() here to declare subsystem dependencies.    
    addRequirements(m_Pneumatics);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Sets Double Solenoid to kReverse
    m_Pneumatics.setSolenoid(Value.kReverse);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
