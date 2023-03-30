// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drivers;

import edu.wpi.first.math.kinematics.SwerveModuleState;

/** Add your docs here. */
public interface SwerveModule {
    double getDriveVelocity();

    double getSteerAngle();

    void set(double driveVoltage, double steerAngle);

    void setDesiredState(SwerveModuleState desiredState);
}
