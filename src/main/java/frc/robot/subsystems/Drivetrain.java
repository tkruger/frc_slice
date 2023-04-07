/// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.*;
import frc.robot.modules.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;

import com.kauailabs.navx.frc.AHRS;

//import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
//import com.swervedrivespecialties.swervelib.SwerveModule;

public class Drivetrain extends SubsystemBase {

  private final SparkMaxSwerveModule leftModuleFront, leftModuleBack, rightModuleFront, rightModuleBack;
  //private final SwerveModule leftSDSModuleFront, leftSDSModuleBack, rightSDSModuleFront, rightSDSModuleBack;

  private final SwerveDriveOdometry m_swerveDrivetrainOdometry;

  private final AHRS navXGyro;

  public final Field2d m_field2d;

  private final Timer autoTrajectoryTimer;

  private Trajectory currentAutoTrajectory;

  /** Creates a new Drivetrain. */
  public Drivetrain() {

    //The gear ratios, motor ports, and steer offsets for these object declarations are placholders for now
    /*leftSDSModuleFront = Mk4iSwerveModuleHelper.createNeo(
      Shuffleboard.getTab("LiveWindow").getLayout("Left Front Module", BuiltInLayouts.kList), 
      Mk4iSwerveModuleHelper.GearRatio.L1, 
      Constants.Drivetrain.LEFT_FRONT_PORT_DRIVE, 
      Constants.Drivetrain.LEFT_FRONT_PORT_STEER, 
      Constants.Drivetrain.LEFT_FRONT_PORT_STEER, 
      0);

    leftSDSModuleBack = Mk4iSwerveModuleHelper.createNeo(
      Shuffleboard.getTab("LiveWindow").getLayout("Left Back Module", BuiltInLayouts.kList), 
      Mk4iSwerveModuleHelper.GearRatio.L1, 
      Constants.Drivetrain.LEFT_BACK_PORT_DRIVE, 
      Constants.Drivetrain.LEFT_BACK_PORT_STEER, 
      Constants.Drivetrain.LEFT_BACK_PORT_STEER, 
      0);

    rightSDSModuleFront = Mk4iSwerveModuleHelper.createNeo(
      Shuffleboard.getTab("LiveWindow").getLayout("Right Front Module", BuiltInLayouts.kList), 
      Mk4iSwerveModuleHelper.GearRatio.L1, 
      Constants.Drivetrain.RIGHT_FRONT_PORT_DRIVE, 
      Constants.Drivetrain.RIGHT_FRONT_PORT_STEER, 
      Constants.Drivetrain.RIGHT_FRONT_PORT_STEER, 
      0);

    rightSDSModuleBack = Mk4iSwerveModuleHelper.createNeo(
      Shuffleboard.getTab("LiveWindow").getLayout("Right Back Module", BuiltInLayouts.kList), 
      Mk4iSwerveModuleHelper.GearRatio.L1, 
      Constants.Drivetrain.RIGHT_BACK_PORT_DRIVE, 
      Constants.Drivetrain.RIGHT_BACK_PORT_STEER, 
      Constants.Drivetrain.RIGHT_BACK_PORT_STEER, 
      0);*/

    //The steer offset arguments for these swerve modules are placeholders for now
    leftModuleFront = new SparkMaxSwerveModule(Constants.kDrivetrain.LEFT_FRONT_PORT_DRIVE, Constants.kDrivetrain.LEFT_FRONT_PORT_STEER, 0);
    leftModuleBack = new SparkMaxSwerveModule(Constants.kDrivetrain.LEFT_BACK_PORT_DRIVE, Constants.kDrivetrain.LEFT_BACK_PORT_STEER, 0);
    rightModuleFront = new SparkMaxSwerveModule(Constants.kDrivetrain.RIGHT_FRONT_PORT_DRIVE, Constants.kDrivetrain.RIGHT_FRONT_PORT_STEER, 0);
    rightModuleBack = new SparkMaxSwerveModule(Constants.kDrivetrain.RIGHT_BACK_PORT_DRIVE, Constants.kDrivetrain.RIGHT_BACK_PORT_STEER, 0);

    navXGyro = new AHRS(SerialPort.Port.kUSB1);

    m_field2d = new Field2d();

    autoTrajectoryTimer = new Timer();

    // Creates and pushes Field2d to SmartDashboard.
    SmartDashboard.putData(m_field2d);

    resetDriveEncoders();

    //These standard deviation values should be measured proplerly for our robot
    m_swerveDrivetrainOdometry = new SwerveDriveOdometry(
      Constants.kDrivetrain.kSwerveKinematics, 
      getRotation2d(), 
      getPositions(),
      new Pose2d(8.28, 4, Rotation2d.fromDegrees(0)));

  }

  @Override
  public void periodic() {
  // This method will be called once per scheduler run

    updateOdometry();

    m_field2d.setRobotPose(getPose());

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  /**
   * Sets the idle mode of all drive motors to either brake mode or coast mode.
   * 
   * @param enableBrakeMode Whether or not the idle mode of all 
   * drive motors should be set to brake mode(false to set to coast mode).
   * 
   */
  public void setDriveIdleMode(boolean enableBrakeMode) {

    leftModuleFront.setDriveIdleMode(enableBrakeMode);
    leftModuleBack.setDriveIdleMode(enableBrakeMode);
    rightModuleFront.setDriveIdleMode(enableBrakeMode);
    rightModuleBack.setDriveIdleMode(enableBrakeMode);

  }

  /**
   * Sets the idle mode of all steer motors to either brake mode or coast mode.
   * 
   * @param enableBrakeMode Whether or not the idle mode of all 
   * steer motors should be set to brake mode(false to set to coast mode).
   * 
   */
  public void setSteerIdleMode(boolean enableBrakeMode) {

    leftModuleFront.setSteerIdleMode(enableBrakeMode);
    leftModuleBack.setSteerIdleMode(enableBrakeMode);
    rightModuleFront.setSteerIdleMode(enableBrakeMode);
    rightModuleBack.setSteerIdleMode(enableBrakeMode);

  }

  /**
   * Sets all drive motors to specified proportional, integral, derivative, and feedforward gains.
   * 
   * @param kP The desired proportional gain for all drive motors to be set to.
   * @param kI The desired integral gain for all drive motors to be set to.
   * @param kD The desired derivative gain for all drive motors to be set to.
   * @param kF The desired feedforward gain for all drive motors to be set to.
   */
  public void setDrivePIDF(double kP, double kI, double kD, double kF) {

    leftModuleFront.setDrivePIDF(kP, kI, kD, kF);
    leftModuleBack.setDrivePIDF(kP, kI, kD, kF);
    rightModuleFront.setDrivePIDF(kP, kI, kD, kF);
    rightModuleBack.setDrivePIDF(kP, kI, kD, kF);

  }

  /**
   * Sets all steer motors to specified proportional, integral, and derivative gains.
   * 
   * @param kP The desired proportional gain for all steer motors to be set to.
   * @param kI The desired integral gain for all steer motors to be set to.
   * @param kD The desired derivative gain for all steer motors to be set to.
   */
  public void setSteerPID(double kP, double kI, double kD) {

    leftModuleFront.setSteerPID(kP, kI, kD);
    leftModuleBack.setSteerPID(kP, kI, kD);
    rightModuleFront.setSteerPID(kP, kI, kD);
    rightModuleBack.setSteerPID(kP, kI, kD);

  }

  /**
   * Sets the maxiumum and reverse power of all native drive PIDF controllers to a specified value.
   * 
   * @param max The desired maximum forward and reverse power to set all native drive PIDF controllers to.
   */
  public void setMaxDriveOutput(double max) {

    leftModuleFront.setMaxDriveOutput(max);
    leftModuleBack.setMaxDriveOutput(max);
    rightModuleFront.setMaxDriveOutput(max);
    rightModuleBack.setMaxDriveOutput(max);

  }

  /**
   * Sets the maxiumum and reverse power of all native steer PID controllers to a specified value.
   * 
   * @param max The desired maximum forward and reverse power to set all native steer PID controllers to.
   */
  public void setMaxSteerOutput(double max) {

    leftModuleFront.setMaxSteerOutput(max);
    leftModuleBack.setMaxSteerOutput(max);
    rightModuleFront.setMaxSteerOutput(max);
    rightModuleBack.setMaxSteerOutput(max);

  }

  /**
   * Drives the robot at given X, Y, and rotational velocities using native PID controllers for the drive
   * and steer motors of the swerve modules.
   * 
   * @param translationX The desired velocity for the robot to move at along the X axis of the field(forwards/backwards from driver POV).
   * @param translationY The desired velocity for the robot to move at along the Y axis of the field(left/right from driver POV).
   * @param rotation The desired velocity for the robot to rotate at.
   */
  public void swerveDrivePID(double translationX, double translationY, double rotation) {

    SwerveModuleState[] states = Constants.kDrivetrain.kSwerveKinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(
      translationX, 
      translationY, 
      rotation, 
      getRotation2d()));

    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.kDrivetrain.kMaxVelocityMetersPerSecond);

    leftModuleFront.setDesiredStatePID(states[0]);
    leftModuleBack.setDesiredStatePID(states[1]);
    rightModuleFront.setDesiredStatePID(states[2]);
    rightModuleBack.setDesiredStatePID(states[3]);

  }

  /**
   * Drives the robot at given X, Y, and rotational velocities using native PID controllers for only the
   * steer motors of the swerve modules.
   * 
   * @param translationX The desired velocity for the robot to move at along the X axis of the field(forwards/backwards from driver POV).
   * @param translationY The desired velocity for the robot to move at along the Y axis of the field(left/right from driver POV).
   * @param rotation The desired velocity for the robot to rotate at.
   */
  public void swerveDrive(double translationX, double translationY, double rotation) {

    SwerveModuleState[] states = Constants.kDrivetrain.kSwerveKinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(
      translationX, 
      translationY, 
      rotation, 
      getRotation2d()));

    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.kDrivetrain.kMaxVelocityMetersPerSecond);

    leftModuleFront.setDesiredState(states[0]);
    leftModuleBack.setDesiredState(states[1]);
    rightModuleFront.setDesiredState(states[2]);
    rightModuleBack.setDesiredState(states[3]);

  }

  /**
   * Drives the robot at given X, Y, and rotational velocities using native PID controllers for only the
   * steer motors of the swerve modules.
   * 
   * @param translationX The desired velocity for the robot to move at along the X axis of the field(forwards/backwards from driver POV).
   * @param translationY The desired velocity for the robot to move at along the Y axis of the field(left/right from driver POV).
   * @param rotation The desired velocity for the robot to rotate at.
   */
  /*public void swerveDrive(double translationX, double translationY, double rotation) {

    SwerveModuleState[] states = Constants.Drivetrain.kSwerveKinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(
      translationX, 
      translationY, 
      rotation, 
      getRotation2d()));

    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Drivetrain.kMaxVelocityMetersPerSecond);

    leftSDSModuleFront.set(states[0].speedMetersPerSecond / Constants.Drivetrain.kMaxVelocityMetersPerSecond * 10, states[0].angle.getRadians());
    leftSDSModuleBack.set(states[1].speedMetersPerSecond / Constants.Drivetrain.kMaxVelocityMetersPerSecond * 10, states[1].angle.getRadians());
    rightSDSModuleFront.set(states[2].speedMetersPerSecond / Constants.Drivetrain.kMaxVelocityMetersPerSecond * 10, states[2].angle.getRadians());
    rightSDSModuleBack.set(states[3].speedMetersPerSecond / Constants.Drivetrain.kMaxVelocityMetersPerSecond * 10, states[3].angle.getRadians());

  }*/

  /**
   * Sends the poses of a desired trajectory to the Field2d object.
   * 
   * @param trajectory The desired trajectory to send to the Field2d object.
   */
  public void setField2d(Trajectory trajectory) {

    // Pushes the trajectory to Field2d.
    m_field2d.getObject("Trajectory").setTrajectory(trajectory);

  }

  /**
   * Resets and starts a timer in order to provide a time 
   * since the beginning of the trajectory to sample from.
   */
  public void startAutoTrajectoryTimer() {

    autoTrajectoryTimer.reset();
    autoTrajectoryTimer.start();

  }

  /**
   * Sets the auto trajectory used to sample the state at each time step from.
   * 
   * @param trajectory The current auto trajectory to sample from.
   */
  public void setCurrentAutoTrajectory(Trajectory trajectory) {

    currentAutoTrajectory = trajectory;
    
  }

  /**
   * Samples and obtains the rotation at the current time step of the current auto trajectory. 
   * 
   * @return The rotation of the robot of at the current time step of the current auto trajectory.
   */
  public Rotation2d getAutoTrajectoryRotation() {

    return currentAutoTrajectory.sample(autoTrajectoryTimer.get()).poseMeters.getRotation();

  }

  /**
   * Updates the drivetrain odometry object to the robot's current position on the field.
   * 
   * @return The new updated pose of the robot.
   */
  public Pose2d updateOdometry() {

    return m_swerveDrivetrainOdometry.update(getRotation2d(), getPositions());

  }

  /**
   * Returns the current pose of the robot without updating
   * the odometry.
   * 
   * @return The current estimated pose of the robot.
   */
  public Pose2d getPose() {
    return m_swerveDrivetrainOdometry.getPoseMeters();
  }

  /**
   * Obtains and returns the current positions of all drivetrain swerve modules.
   * 
   * @return The current positions of all drivetrain swerve modules.
   */
  public SwerveModulePosition[] getPositions() {

    SwerveModulePosition[] positions = {
      leftModuleFront.getPosition(),
      leftModuleBack.getPosition(),
      rightModuleFront.getPosition(),
      rightModuleBack.getPosition()};

    return positions;

  }

  /**
   * Obtains and returns the current states of all drivetrain swerve modules.
   * 
   * @return The current states of all drivetrain swerve modules.
   */
  public SwerveModuleState[] getStates() {

    SwerveModuleState[] states = {
      leftModuleFront.getState(),
      leftModuleBack.getState(),
      rightModuleFront.getState(),
      rightModuleBack.getState()};

      return states;

    }

  /**
   * Obtains and returns the target states that the drivetrain swerve modules have been set to.
   * 
   * @return The target states that the drivetrain swerve modules have been set to.
   */
  public SwerveModuleState[] getTargetStates() {

    SwerveModuleState[] targetStates = {
      leftModuleFront.getTargetState(),
      leftModuleBack.getTargetState(),
      rightModuleFront.getTargetState(),
      rightModuleBack.getTargetState()};

    return targetStates;
    
  }

  /**
   * Resets the position of the odometry object using a specified position.
   * 
   * @param position The desired position to reset the odometry of the robot to.
   */
  public void resetOdometry(Pose2d position) {

    m_swerveDrivetrainOdometry.resetPosition(getRotation2d(), getPositions(), position);

  }

  /**
   * Sets the positions of all drive encoders to 0(meters).
   */
  public void resetDriveEncoders() {

    leftModuleFront.resetDriveEncoder();
    leftModuleBack.resetDriveEncoder();
    rightModuleFront.resetDriveEncoder();
    rightModuleBack.resetDriveEncoder();

  }

  /**
   * Obtains and returns the current heading of the robot as a Rotation2d from the gyro object.
   * 
   * @return The current heading of the robot as a Rotation2d.
   */
  public Rotation2d getRotation2d() {

    return navXGyro.getRotation2d();
    
  }
  
  /**
   * Obtains and returns the current heading of the robot going positive counter-clockwise from 0 to 360 degrees from the gyro object.
   *
   * @return The current heading of the robot going counter-clockwise positive from 0 to 360 degrees.
   */
  public double getHeading() {

    return -navXGyro.getYaw() + 180;

  }

  /**
   * Obtains and returns the current pitch of the robot from -180 to 180 degrees, with an offset of 1 degree from the gyro object.
   * 
   * @return The current pitch of the robot from -180 to 180 degrees, with an offset of 1 degree.
   */
  public double getPitch() {

    return navXGyro.getPitch() + 1;

  }

  /**
   * Obtains and returns the current roll of the robot from -180 to 180 degrees, with an offset of 1.7 degrees from the gyro object.
   * 
   * @return The current pitch of the robot from -180 to 180 degrees, with an offset of 1.7 degrees.
   */
  public double getRoll() {

    return navXGyro.getRoll() + 1.7;

  }

  /**
   * Resets the gyro yaw axis to a heading of 0.
   */
  public void resetHeading() {

    navXGyro.reset();

  }

  /**
   * Sets the desired states of all drivetrain swerve modules to a specified arrary of states.
   * 
   * @param states The desired states for all drivetrian swerve modules to be set to.
   */
  public void setModuleStates(SwerveModuleState[] states) {

    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.kDrivetrain.kMaxVelocityMetersPerSecond);

    leftModuleFront.setDesiredState(states[0]);
    leftModuleBack.setDesiredState(states[1]);
    rightModuleFront.setDesiredState(states[2]);
    rightModuleBack.setDesiredState(states[3]);

  }

  /**
   * Sets the desired states of all drivetrain swerve modules to a specified arrary of states.
   * 
   * @param states The desired states for all drivetrian swerve modules to be set to.
   */
  /*public void setSDSModuleStates(SwerveModuleState[] states) {

    //The maximum voltage value of 10 is taken from Trajectories
    leftSDSModuleFront.set(states[0].speedMetersPerSecond / Constants.Drivetrain.kMaxVelocityMetersPerSecond * 10, states[0].angle.getRadians());
    leftSDSModuleBack.set(states[1].speedMetersPerSecond / Constants.Drivetrain.kMaxVelocityMetersPerSecond * 10, states[1].angle.getRadians());
    rightSDSModuleFront.set(states[2].speedMetersPerSecond / Constants.Drivetrain.kMaxVelocityMetersPerSecond * 10, states[2].angle.getRadians());
    rightSDSModuleBack.set(states[3].speedMetersPerSecond / Constants.Drivetrain.kMaxVelocityMetersPerSecond * 10, states[2].angle.getRadians());

  }*/

  /**
   * Sets all drivetrain swerve modules to states with speeds of 0 and the current angles of the modules.
   */
  public void stopDrive() {

    SwerveModuleState[] stopStates = {
      new SwerveModuleState(0, leftModuleFront.getState().angle),
      new SwerveModuleState(0, leftModuleBack.getState().angle),
      new SwerveModuleState(0, rightModuleFront.getState().angle),
      new SwerveModuleState(0, rightModuleBack.getState().angle)};


    setModuleStates(stopStates);
    
  }

}
