// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Limelight;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class LimelightAlignmentCommand extends CommandBase {

  private final Limelight m_limelight;
  private final Drivetrain m_drivetrain;

  private Pose2d botPose;
  private double rotation;
  private double targetDetected;
  private double aprilTagID;

  /** Creates a new LimelightAlignmentCommand. */
  public LimelightAlignmentCommand(Limelight limelight, Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(limelight, drivetrain);

    m_limelight = limelight;
    m_drivetrain = drivetrain;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    botPose = m_limelight.getBotPose();
    rotation = botPose.getRotation().getDegrees();
    targetDetected = m_limelight.getTargetDetected();

    if(botPose != null && targetDetected == 1) {

      if(rotation >= 0 && rotation < 178) {

        m_drivetrain.curvatureDrive(0, Constants.limelight_X_ALIGNMENT_SPEED);

      }
      else if(rotation <= 0 && rotation > -178) {

        m_drivetrain.curvatureDrive(0, -Constants.limelight_X_ALIGNMENT_SPEED);

      }

    }

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
