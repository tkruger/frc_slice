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

  private final Drivetrain m_drivetrain;
  private final Limelight m_limelight;

  /*private Pose2d lastBotPose;
  private Pose2d currentBotPose;
  private double rotation;
  private double lastAprilTagID;
  private double currentAprilTagID;*/

  private Pose2d robotTargetSpacePose;
  private double targetSpaceRotation;

  private Pose2d lastBotPose;
  private Pose2d currentBotPose;

  private double botPoseRotation;

  /** Creates a new LimelightAlignmentCommand. */
  public LimelightAlignmentCommand(Limelight limelight, Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(limelight, drivetrain);

    m_drivetrain = drivetrain;
    m_limelight = limelight;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    /*currentBotPose = Limelight.getBotPose();

    if(currentBotPose != null) {

      lastBotPose = currentBotPose;

    }

    rotation = lastBotPose.getRotation().getDegrees();

    currentAprilTagID = m_limelight.getAprilTagID();
    
    if(currentAprilTagID != 0) {

      lastAprilTagID = currentAprilTagID;

    }

    if(lastAprilTagID == 6) {

      if(lastBotPose.getY() > 4.51621) {

        if(rotation > -85 && rotation < 85) {

          m_drivetrain.ArcadeDrive(0, -Constants.limelight_TURN_ALIGNMENT_SPEED);

        }
        else if(rotation > 95 && rotation < -95) {

          m_drivetrain.ArcadeDrive(0, Constants.limelight_TURN_ALIGNMENT_SPEED);

        }
        else {

          m_drivetrain.ArcadeDrive(Constants.limelight_FORWARD_ALIGNMENT_SPEED, 0);

        }

      }

      else if(lastBotPose.getY() < 4.31621) {

        if(rotation > -85 && rotation < 85) {

          m_drivetrain.ArcadeDrive(0, Constants.limelight_TURN_ALIGNMENT_SPEED);

        }
        else if(rotation > 95 && rotation < -95) {

          m_drivetrain.ArcadeDrive(0, -Constants.limelight_TURN_ALIGNMENT_SPEED);

        }
        else {

          m_drivetrain.ArcadeDrive(Constants.limelight_FORWARD_ALIGNMENT_SPEED, 0);

        }

      }

      else if(rotation > 5 && rotation < 90) {

        m_drivetrain.ArcadeDrive(0, -Constants.limelight_TURN_ALIGNMENT_SPEED);

      }
      else if(rotation < -5 && rotation > -90) {

        m_drivetrain.ArcadeDrive(0, Constants.limelight_TURN_ALIGNMENT_SPEED);

      }
      else {

        m_drivetrain.ArcadeDrive(Constants.limelight_FORWARD_ALIGNMENT_SPEED, 0);

      }

    }*/

    robotTargetSpacePose = m_limelight.getRobotTargetSpacePose();
    targetSpaceRotation = m_limelight.getRobotTargetSpacePose().getRotation().getDegrees();

    currentBotPose = Limelight.getBotPose();

    if(currentBotPose != null) {

      lastBotPose = currentBotPose;

    }

    botPoseRotation = lastBotPose.getRotation().getDegrees();

    if(robotTargetSpacePose.getY() > 0.1) {

      if(targetSpaceRotation > -85 && targetSpaceRotation <= 90) {

        m_drivetrain.ArcadeDrive(0, Constants.limelight_TURN_ALIGNMENT_SPEED);
  
      }
      else if(targetSpaceRotation > 90 && targetSpaceRotation < -95) {
  
        m_drivetrain.ArcadeDrive(0, -Constants.limelight_TURN_ALIGNMENT_SPEED);
  
      }
      else {

        m_drivetrain.ArcadeDrive(Constants.limelight_FORWARD_ALIGNMENT_SPEED, 0);

      }

    }

    else if(robotTargetSpacePose.getY() < -0.1) {

      if(targetSpaceRotation >= -90 && targetSpaceRotation < 85) {

        m_drivetrain.ArcadeDrive(0, -Constants.limelight_TURN_ALIGNMENT_SPEED);
  
      }
      else if(targetSpaceRotation > 95 && targetSpaceRotation < -90) {
  
        m_drivetrain.ArcadeDrive(0, Constants.limelight_TURN_ALIGNMENT_SPEED);
  
      }
      else {

        m_drivetrain.ArcadeDrive(Constants.limelight_FORWARD_ALIGNMENT_SPEED, 0);

      }

    }

    else if(targetSpaceRotation <= 0 && targetSpaceRotation > -175) {

      m_drivetrain.ArcadeDrive(0, Constants.limelight_TURN_ALIGNMENT_SPEED);

    }
    else if(targetSpaceRotation < 175 && targetSpaceRotation > 0) {

      m_drivetrain.ArcadeDrive(0, -Constants.limelight_TURN_ALIGNMENT_SPEED);

    }
    else {

      m_drivetrain.ArcadeDrive(Constants.limelight_FORWARD_ALIGNMENT_SPEED, 0);

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
