package frc.robot.auto.paths;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

import frc.robot.AutoSelector;
import frc.robot.auto.AutoPaths;
import frc.robot.subsystems.Drivetrain;

public class PlaceOneGamePiecePath extends AutoPaths{
    
    public PlaceOneGamePiecePath(AutoSelector.StartingPosition startPosition, Drivetrain drive) {

        switch(startPosition) {

            case BLUE_COMMUNITY_LEFT:
                trajectory = PathPlanner.loadPath("Blue Left Place One Game Piece", new PathConstraints(0.5, 0.2));
            case BLUE_COMMUNITY_MIDDLE:
                trajectory = PathPlanner.loadPath("Blue Middle Place One Game Piece", new PathConstraints(0.5, 0.2));
            case BLUE_COMMUNITY_RIGHT:
                trajectory = PathPlanner.loadPath("Blue Right Place One Game Piece", new PathConstraints(0.5, 0.2));
            case RED_COMMUNITY_LEFT:
                trajectory = PathPlanner.loadPath("Red Left Place One Game Piece", new PathConstraints(0.5, 0.2));
            case RED_COMMUNITY_MIDDLE:
                trajectory = PathPlanner.loadPath("Red Middle Place One Game Piece", new PathConstraints(0.5, 0.2));
            case RED_COMMUNITY_RIGHT:
                trajectory = PathPlanner.loadPath("Red Right Place One Game Piece", new PathConstraints(0.5, 0.2));
            default:
                break;

        }

    }

}
