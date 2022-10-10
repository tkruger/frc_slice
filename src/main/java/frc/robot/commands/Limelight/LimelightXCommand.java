// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Limelight;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import edu.wpi.first.wpilibj.Timer; 

public class LimelightXCommand extends CommandBase {
  /** Creates a new LimelightRun. */

  private final Limelight m_limelight;

  private final Drivetrain m_drivetrain;

  static double targetDetected;
  static double targetXOffset;
  static double targetYOffset;

  double xSteeringAdjust;
  double ySteeringAdjust;

  private Timer Time;

  boolean finished;

  public LimelightXCommand(Limelight limelight, Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(limelight, drivetrain);

    this.m_limelight = limelight;
    this.m_drivetrain = drivetrain;

    Time.reset();
    Time.start();

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_limelight.setCameraMode(1);
    finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    targetDetected = m_limelight.getTargetDetected();
    targetXOffset = m_limelight.getXOffset();
    targetYOffset = m_limelight.getYOffset();

    if(targetDetected == 1) {

      if (targetXOffset > 8) {
        xSteeringAdjust = Constants.limelight_STEERING_ADJUST_PROPORTION * 8;
      } else if (targetXOffset < -8) {
        xSteeringAdjust = Constants.limelight_STEERING_ADJUST_PROPORTION * -8;
      } else if (targetXOffset < 5) {
        xSteeringAdjust = Constants.limelight_STEERING_ADJUST_PROPORTION * 4;
      } else if (targetXOffset > -5) {
        xSteeringAdjust = Constants.limelight_STEERING_ADJUST_PROPORTION * -4;
      } else {
        xSteeringAdjust = Constants.limelight_STEERING_ADJUST_PROPORTION * targetXOffset;
      }
      
      m_drivetrain.ArcadeDrive(0, xSteeringAdjust);

    } else {
      ySteeringAdjust = 0;
      xSteeringAdjust = 19 * Constants.limelight_STEERING_ADJUST_PROPORTION;

      m_drivetrain.ArcadeDrive(ySteeringAdjust, xSteeringAdjust);
    }

    if (targetDetected == 1 && java.lang.Math.abs(targetXOffset) < 4.2 && java.lang.Math.abs(m_drivetrain.getTurnRate()) < 0.8) {
      finished = true;
    }

    SmartDashboard.putNumber("Limelight Target Detected:", targetDetected);
    SmartDashboard.putNumber("Limelight X Offset", targetXOffset);
    SmartDashboard.putNumber("Limelight Y Offset", targetYOffset);

    m_limelight.setLedMode(3);
    m_limelight.setCameraMode(0);
    m_limelight.setPipeline();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_limelight.setCameraMode(1);

    targetXOffset = 0;
    targetYOffset = 0;
    targetDetected = 0;

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Time.get() >= 8) {
      return true;
    }

    return finished;

  }
}
