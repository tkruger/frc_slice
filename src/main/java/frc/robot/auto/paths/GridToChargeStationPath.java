package frc.robot.auto.paths;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

import frc.robot.AutoSelector;
import frc.robot.auto.AutoPaths;
import frc.robot.subsystems.Drivetrain;

public class GridToChargeStationPath extends AutoPaths{
    
    public GridToChargeStationPath(AutoSelector.StartingPosition startPosition, Drivetrain drive) {

        switch(startPosition) {

            case BLUE_COMMUNITY_LEFT:
                trajectory = PathPlanner.loadPath("Blue Left Grid To Charge Station Path", new PathConstraints(4, 1));
            case BLUE_COMMUNITY_MIDDLE:
                trajectory = PathPlanner.loadPath("Blue Middle Grid To Charge Station Path", new PathConstraints(4, 1));
            case BLUE_COMMUNITY_RIGHT:
                trajectory = PathPlanner.loadPath("Blue Right Grid To Charge Station Path", new PathConstraints(4, 1));
            case RED_COMMUNITY_LEFT:
                trajectory = PathPlanner.loadPath("Red Left Grid To Charge Station Path", new PathConstraints(4, 1));
            case RED_COMMUNITY_MIDDLE:
                trajectory = PathPlanner.loadPath("Red Middle Grid To Charge Station Path", new PathConstraints(4, 1));
            case RED_COMMUNITY_RIGHT:
                trajectory = PathPlanner.loadPath("Red Right Grid To Charge Station Path", new PathConstraints(4, 1));
            default:
                break;

        }

    }

}
