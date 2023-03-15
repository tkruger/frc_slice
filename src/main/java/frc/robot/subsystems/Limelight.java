// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {

  private final NetworkTable table;
  
  private double targetDetected;

  private double targetXOffset;
  private double targetYOffset;

  private static double[] currentBotPoseBlue;
  private static double[] lastBotPoseBlue;
  private double[] lastRobotTargetSpacePose;
  private double[] currentRobotTargetSpacePose;

  private double aprilTagID;

  private final NetworkTableEntry ledMode;

  private final NetworkTableEntry cameraMode;

  private final NetworkTableEntry pipeline;

  /** Creates a new Limelight. */
  public Limelight() {

    table = NetworkTableInstance.getDefault().getTable("limelight-slice");

    ledMode = table.getEntry("ledMode");
    cameraMode = table.getEntry("cameraMode");
    pipeline = table.getEntry("pipeline");

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    targetDetected = table.getEntry("tv").getDouble(0);

    targetXOffset = table.getEntry("tx").getDouble(0);
    targetYOffset = table.getEntry("ty").getDouble(0);

    currentBotPoseBlue = table.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
    currentRobotTargetSpacePose = table.getEntry("botpose_targetspace").getDoubleArray(new double[6]);

    if(currentRobotTargetSpacePose != null) {

      lastRobotTargetSpacePose = currentRobotTargetSpacePose;

    }

    aprilTagID = table.getEntry("tid").getDouble(0);

  }

  public double getTargetDetected() {

    return targetDetected;

  }

  public double getXOffset() {

    return targetXOffset;

  }

  public double getYOffset() {

    return targetYOffset;

  }

  public static Pose2d getBotPoseBlue() {

    lastBotPoseBlue = currentBotPoseBlue;

    if(lastBotPoseBlue != null) {

      return new Pose2d(lastBotPoseBlue[0], lastBotPoseBlue[1], Rotation2d.fromDegrees(lastBotPoseBlue[5]));

    }
    else {

      return null;

    }

  }

  public Pose2d getRobotTargetSpacePose() {

    return new Pose2d(lastRobotTargetSpacePose[0], lastRobotTargetSpacePose[1], Rotation2d.fromDegrees(lastRobotTargetSpacePose[5]));

  }

  public double getAprilTagID() {

    return aprilTagID;

  }

  public void setLedMode(Number mode) {

    ledMode.setNumber(mode);

  }
  
  public void setCameraMode(Number mode) {

    cameraMode.setNumber(mode);

  }

  public void setPipeline(Number pipelineNumber) {

    pipeline.setNumber(pipelineNumber);

  }

  public Trajectory generateDoubleSubstationTrajectory(Pose2d initialPosition) {

    double aprilTagX;
    double aprilTagY;

    Pose2d finalPosition;

    if(aprilTagID == 4 || aprilTagID == 5) {

      if(aprilTagID == 4) {

        aprilTagX = 16.19;
        aprilTagY = 6.74;
        finalPosition = new Pose2d(aprilTagX - 1.2, aprilTagY, Rotation2d.fromDegrees(0));
  
      }
      else {
  
        aprilTagX = 0.37;
        aprilTagY = 6.74;
        finalPosition = new Pose2d(aprilTagX + 1.2, aprilTagY, Rotation2d.fromDegrees(180));
  
      }
  
      return TrajectoryGenerator.generateTrajectory(initialPosition, 
      List.of(new Translation2d(
        (initialPosition.getX() + finalPosition.getX()) / 2, 
        (initialPosition.getY() + finalPosition.getY()) / 2)),
      finalPosition, 
      new TrajectoryConfig(0.5, 0.2));
  
    }
    else {

      return new Trajectory();

    }

  }

}