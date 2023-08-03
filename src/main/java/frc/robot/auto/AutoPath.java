// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;

/** 
 * This class serves as a superclass for all auto path classes primarily to
 * provide them all with a common type that can be used for things such as
 * parameters.
*/
public class AutoPath {

    public Trajectory trajectory;

    public Pose2d getPathStartingPosition() {

        return trajectory.getInitialPose();

    }

}