// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain.ChargeStation;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.cLEDs;
import frc.robot.subsystems.*;

public class ChargeStationBalancePIDCommand extends CommandBase {

  private final Drivetrain m_drivetrain;
  private final LEDs m_leds;

  private double pitch;

  private final PIDController pidController;

  /** Creates a new ChargeStationBalanceCommand. */
  public ChargeStationBalancePIDCommand(Drivetrain drivetrain, LEDs leds) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain, leds);

    m_drivetrain = drivetrain;
    m_leds = leds;

    //pidController = new PIDController(0.032, 0.0001, 0.0009);
    pidController = new PIDController(0.032, 0.0001, 0.0009);
    pidController.setSetpoint(0);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_drivetrain.setIdleMode(true);

    m_drivetrain.setPIDF(.17, .000001, .12, .62);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    pitch = m_drivetrain.getRoll();

    //if (Math.abs(pitch) > 2) {
    if (Math.abs(pitch) > 0.5) {  
      m_drivetrain.PIDArcadeDrive(pidController.calculate(pitch), 0);
    } else {
      m_drivetrain.PIDArcadeDrive(0, 0);
      pidController.reset();
    }

    m_leds.balanceLedMode(pitch);

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
