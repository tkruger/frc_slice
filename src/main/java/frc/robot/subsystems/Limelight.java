// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */

  private final NetworkTable table;
  
  private static double targetDetected;

  private double targetXOffset;
  private double targetYOffset;

  private static double[] currentBotPose;
  private static double[] lastBotPose;

  private double[] lastRobotTargetSpacePose;
  private double[] currentRobotTargetSpacePose;

  private static double aprilTagID;

  private final NetworkTableEntry ledMode;

  private final NetworkTableEntry cameraMode;

  private final NetworkTableEntry pipeline;

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

    currentBotPose = table.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
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

  public static Pose2d getBotPose() {

    lastBotPose = currentBotPose;

    if(lastBotPose != null) {

      return new Pose2d(lastBotPose[0], lastBotPose[1], Rotation2d.fromDegrees(lastBotPose[5]));

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

}