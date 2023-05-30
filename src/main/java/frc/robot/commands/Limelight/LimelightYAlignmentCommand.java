// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Limelight;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.Timer; 

public class LimelightYAlignmentCommand extends CommandBase {

  private final Limelight m_limelight;
  private final Drivetrain m_drivetrain;
  private final double pipeline;
  private double aprilTag = -1;

  private double targetDetected;
  private double targetYOffset;
  private double currentApriltag = -1;

  private double ySteeringAdjust;

  private final Timer Time;

  boolean finished;

  /** Creates a new LimelightXAlignmentCommand for a cube. */
  public LimelightYAlignmentCommand(Limelight limelight, Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(limelight, drivetrain);

    this.m_limelight = limelight;
    this.m_drivetrain = drivetrain;
    pipeline = 0;

    Time = new Timer();

  }

  /** Creates a new LimelightXAlignmentCommand for an Apriltag. */
  public LimelightYAlignmentCommand(Limelight limelight, Drivetrain drivetrain, double aprilTag) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(limelight, drivetrain);

    this.m_limelight = limelight;
    this.m_drivetrain = drivetrain;
    this.aprilTag = aprilTag;
    pipeline = 1;

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

    m_limelight.setPipeline(pipeline);
    targetDetected = m_limelight.getTargetDetected();
    targetYOffset = m_limelight.getYOffset();
    if(aprilTag > 0) currentApriltag = m_limelight.getAprilTagID();


    if(targetDetected == 1 && (aprilTag < 1 || currentApriltag == aprilTag)) {

      if(targetYOffset > 0) {
        m_drivetrain.ArcadeDrive(-.4, 0);
      } else {
        m_drivetrain.ArcadeDrive(.4, 0);
      }

    }

    if (targetDetected == 1 && java.lang.Math.abs(targetYOffset) < 2) {
      finished = true;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    targetYOffset = 0;
    targetDetected = 0;

    m_drivetrain.ArcadeDrive(0, 0);

    m_limelight.setPipeline(1);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Time.get() >= 7) {
      return true;
    }

    return finished;

  }
  
}