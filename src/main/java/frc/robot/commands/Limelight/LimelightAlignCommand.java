// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Limelight;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class LimelightAlignCommand extends CommandBase {

  private final Drivetrain m_drivetrain;
  private final Limelight m_limelight;

  /*private Pose2d robotTargetSpacePose;
  private double targetSpaceRotation;

  private Pose2d lastBotPose;
  private Pose2d currentBotPose;

  private double botPoseRotation;*/

  private Pose2d position;
  private double heading;

  private double aprilTagX;
  private double aprilTagY;

  private double lastAprilTagID;
  private double currentAprilTagID;

  /** Creates a new LimelightAlignmentCommand. */
  public LimelightAlignCommand(Limelight limelight, Drivetrain drivetrain) {
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

    //This code can hypothetically only run when the robot sees the AprilTag
    /*robotTargetSpacePose = m_limelight.getRobotTargetSpacePose();
    targetSpaceRotation = m_limelight.getRobotTargetSpacePose().getRotation().getDegrees();

    currentBotPose = Limelight.getBotPoseBlue();

    if(currentBotPose != null) {

      lastBotPose = currentBotPose;

    }

    botPoseRotation = lastBotPose.getRotation().getDegrees();

    //The turn speeds used below may need to be reversed to have intended functionality
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

    }*/

    //This code can hypothetically function without the robot constantly seeing the AprilTag
    position = m_drivetrain.getPose();
    heading = position.getRotation().getDegrees();

    currentAprilTagID = m_limelight.getAprilTagID();

    if(currentAprilTagID != 0) {

      lastAprilTagID = currentAprilTagID;

    }

    if(lastAprilTagID == 1) {

      aprilTagX = 15.52;
      aprilTagY = 1.06;

    }
    else if(lastAprilTagID == 2) {

      aprilTagX = 15.52;
      aprilTagY = 2.73;

    }
    else if(lastAprilTagID == 3) {

      aprilTagX = 15.52;
      aprilTagY = 4.42;

    }
    else if(lastAprilTagID == 6) {

      aprilTagX = 1.04;
      aprilTagY = 4.42;
    }
    else if(lastAprilTagID == 7) {

      aprilTagX = 1.04;
      aprilTagY = 2.73;
    }
    else if(lastAprilTagID == 8) {

      aprilTagX = 1.04;
      aprilTagY = 1.06;

    }
    
    //The turn speeds used below may need to be reversed to have intended functionality
    if(position.getY() > (aprilTagY + 0.1)) {

      if(heading > -85 && heading <= 90) {

        m_drivetrain.ArcadeDrive(0, Constants.limelight_TURN_ALIGNMENT_SPEED);
  
      }
      else if(heading > 90 && heading < -95) {
  
          m_drivetrain.ArcadeDrive(0, -Constants.limelight_TURN_ALIGNMENT_SPEED);
  
      }
      else {

        m_drivetrain.ArcadeDrive(Constants.limelight_FORWARD_ALIGNMENT_SPEED, 0);

      }

    }

    else if(position.getY() < (aprilTagY - 0.1)) {

      if(heading >= -90 && heading < 85) {

        m_drivetrain.ArcadeDrive(0, -Constants.limelight_TURN_ALIGNMENT_SPEED);
  
      }
      else if(heading > 95 && heading < -90) {
  
        m_drivetrain.ArcadeDrive(0, Constants.limelight_TURN_ALIGNMENT_SPEED);
  
      }
      else {

        m_drivetrain.ArcadeDrive(Constants.limelight_FORWARD_ALIGNMENT_SPEED, 0);

      }

    }

    else if(position.getX() > aprilTagX) {

      if(heading <= 0 && heading > -175) {

        m_drivetrain.ArcadeDrive(0, Constants.limelight_TURN_ALIGNMENT_SPEED);

      }
      else if(heading < 175 && heading > 0) {

        m_drivetrain.ArcadeDrive(0, -Constants.limelight_TURN_ALIGNMENT_SPEED);

      }
      else {

        m_drivetrain.ArcadeDrive(Constants.limelight_FORWARD_ALIGNMENT_SPEED, 0);
  
      }

    }
    else if(position.getX() < aprilTagX) {

      if(heading < -5 && heading >= -180) {

        m_drivetrain.ArcadeDrive(0, -Constants.limelight_TURN_ALIGNMENT_SPEED);

      }
      else if(heading <= 180 && heading > 5) {

        m_drivetrain.ArcadeDrive(0, Constants.limelight_TURN_ALIGNMENT_SPEED);

      }
      else {

        m_drivetrain.ArcadeDrive(Constants.limelight_FORWARD_ALIGNMENT_SPEED, 0);
  
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
