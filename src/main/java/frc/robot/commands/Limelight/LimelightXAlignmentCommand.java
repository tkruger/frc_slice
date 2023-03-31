// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Limelight;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.Timer; 

public class LimelightXAlignmentCommand extends CommandBase {

  private final Limelight m_limelight;
  private final Drivetrain m_drivetrain;

  private double targetDetected;
  private double targetXOffset;

  private double xSteeringAdjust;

  private final Timer Time;

  boolean finished;

  /** Creates a new LimelightXAlignmentCommand. */
  public LimelightXAlignmentCommand(Limelight limelight, Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(limelight, drivetrain);

    this.m_limelight = limelight;
    this.m_drivetrain = drivetrain;

    Time = new Timer();

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_limelight.setCameraMode(0);
    m_limelight.setLedMode(3);
    finished = false;

    Time.reset();
    Time.start();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    targetDetected = m_limelight.getTargetDetected();
    targetXOffset = m_limelight.getXOffset();

    if(targetDetected == 1) {

      if (targetXOffset > 10) {
        xSteeringAdjust = Constants.Limelight.STEERING_ADJUST_PROPORTION * 14;
      } else if (targetXOffset < -10) {
        xSteeringAdjust = Constants.Limelight.STEERING_ADJUST_PROPORTION * -14;
      } else if (targetXOffset <= 10 && targetXOffset > 0) {
        xSteeringAdjust = Constants.Limelight.STEERING_ADJUST_PROPORTION * 10;
      } else if (targetXOffset >= -10 && targetXOffset < 0) {
        xSteeringAdjust = Constants.Limelight.STEERING_ADJUST_PROPORTION * -10;
      } else {
        xSteeringAdjust = Constants.Limelight.STEERING_ADJUST_PROPORTION * targetXOffset;
      }

      m_drivetrain.ArcadeDrive(0, xSteeringAdjust);

    } else {
      xSteeringAdjust = 17 * Constants.Limelight.STEERING_ADJUST_PROPORTION;

      m_drivetrain.ArcadeDrive(0, xSteeringAdjust);
    }

    if (targetDetected == 1 && java.lang.Math.abs(targetXOffset) < 2 && java.lang.Math.abs(m_drivetrain.getTurnRate()) < 0.8) {
      finished = true;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    targetXOffset = 0;
    targetDetected = 0;

    m_drivetrain.ArcadeDrive(0, 0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Time.get() >= 3) {
      return true;
    }

    return finished;

  }
  
}