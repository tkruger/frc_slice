// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.paths;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

import frc.robot.auto.AutoPaths;
import frc.robot.auto.AutoSelector;

/** Add your docs here. */
public class GridOutOfCommunityToChargeStationPath extends AutoPaths{

    public GridOutOfCommunityToChargeStationPath(AutoSelector.StartingPosition startPosition) {

        switch(startPosition) {

            case BLUE_COMMUNITY_LEFT:
                trajectory = PathPlanner.loadPath("Blue Left Grid Out Of Community To Charge Station", new PathConstraints(0.5, 0.2), true);
                break;
            case BLUE_COMMUNITY_MIDDLE:
                trajectory = PathPlanner.loadPath("Blue Middle Grid Out Of Community To Charge Station", new PathConstraints(0.5, 0.2), true);
                break;
            case BLUE_COMMUNITY_RIGHT:
                trajectory = PathPlanner.loadPath("Blue Right Grid Out Of Community To Charge Station", new PathConstraints(0.5, 0.2), true);
                break;
            case RED_COMMUNITY_LEFT:
                trajectory = PathPlanner.loadPath("Red Left Grid Out Of Community To Charge Station", new PathConstraints(0.5, 0.2), true);
                break;
            case RED_COMMUNITY_MIDDLE:
                trajectory = PathPlanner.loadPath("Red Middle Grid Out Of Community To Charge Station", new PathConstraints(0.5, 0.2), true);
                break;
            case RED_COMMUNITY_RIGHT:
                trajectory = PathPlanner.loadPath("Red Right Grid Out Of Community To Charge Station", new PathConstraints(0.5, 0.2), true);
                break;
            default:
                break;

        }

    }

}
