package frc.robot.auto;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.*;

public class Paths {

    static Trajectory blue1Trajectory1;
    static Path blue1TrajectoryPath1;
    static String blue1Trajectory1JSON;

    static Trajectory blue1Trajectory2;
    static Path blue1TrajectoryPath2;
    static String blue1Trajectory2JSON;

    static Trajectory blue1Trajectory3;
    static Path blue1TrajectoryPath3;
    static String blue1Trajectory3JSON;

    static Trajectory returnTrajectory;

    private static ShuffleboardTab smartDashboardTab;
    private static Double autoPathSelection;
    private static SimpleWidget autoPathWidget;

    public static Trajectory getAutoPath(int pathNumber) {

        try {
            smartDashboardTab = Shuffleboard.getTab("SmartDashboard");
            autoPathWidget = smartDashboardTab.add("Auto Selector", 1).withPosition(4, 2);
        } catch (Exception e) {
            System.out.println("autoPathWidget Caught");
        }
        
        autoPathSelection = autoPathWidget.getEntry().getDouble(1);

        //Path string variable declarations
        blue1Trajectory1JSON = "output/Blue 1 Path 1.wpilib.json";
        blue1Trajectory2JSON = "output/Blue 1 Path 2.wpilib.json";
        blue1Trajectory3JSON = "output/Blue 1 Path 3.wpilib.json";

        //Trajectory object declarations
        blue1Trajectory1 = new Trajectory();
        blue1TrajectoryPath1 = Filesystem.getDeployDirectory().toPath().resolve(blue1Trajectory1JSON);
        blue1Trajectory2 = new Trajectory();
        blue1TrajectoryPath2 = Filesystem.getDeployDirectory().toPath().resolve(blue1Trajectory2JSON);
        blue1Trajectory3 = new Trajectory();
        blue1TrajectoryPath3 = Filesystem.getDeployDirectory().toPath().resolve(blue1Trajectory3JSON);

        try {
            blue1Trajectory1 = TrajectoryUtil.fromPathweaverJson(blue1TrajectoryPath1);
        } 
        catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + blue1Trajectory1JSON, ex.getStackTrace());
        }
        try {
            blue1Trajectory2 = TrajectoryUtil.fromPathweaverJson(blue1TrajectoryPath2);
        }
        catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + blue1Trajectory2JSON, ex.getStackTrace());
        }
        try {
            blue1Trajectory3 = TrajectoryUtil.fromPathweaverJson(blue1TrajectoryPath3);
        }
        catch(IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + blue1Trajectory3JSON, ex.getStackTrace());
        }

        /*Trajectory selection for autonomous
        (Many of these return statements use blue1Trajectory1 as a placeholder for now until more trajectory objects are created)*/
        autoPathSelection = 1.0;
        if(autoPathSelection == 1.0) {

            if(pathNumber == 1) {
                returnTrajectory = blue1Trajectory1;
            }
            else if(pathNumber == 2) {
                returnTrajectory = blue1Trajectory2;
            }
            else if(pathNumber == 3) {
                returnTrajectory = blue1Trajectory3;
            }
        }

        else if(autoPathSelection == 2.0) {

            if(pathNumber == 1) {
                returnTrajectory = blue1Trajectory1;
            }
            else if(pathNumber == 2) {
                returnTrajectory = blue1Trajectory1;
            }
            else if(pathNumber == 3)
                returnTrajectory = blue1Trajectory1;

        }

        else if(autoPathSelection == 3.0) {

            if(pathNumber == 1) {
                returnTrajectory = blue1Trajectory1;
            }
            else if(pathNumber == 2) {
                returnTrajectory = blue1Trajectory1;
            }
            else if(pathNumber == 3) {
                returnTrajectory = blue1Trajectory1;
            }

        }

        else if(autoPathSelection == 4.0) {

            if(pathNumber == 1) {
                returnTrajectory = blue1Trajectory1;
            }
            else if(pathNumber == 2) {
                returnTrajectory = blue1Trajectory1;
            }
            else if(pathNumber == 3) {
                returnTrajectory = blue1Trajectory1;
            }
        }

        else if(autoPathSelection == 5.0) {

            if(pathNumber == 1) {
                returnTrajectory = blue1Trajectory1;
            }
            else if(pathNumber == 2) {
                returnTrajectory = blue1Trajectory1;
            }
            else if(pathNumber == 3) {
                returnTrajectory = blue1Trajectory1;
            }
        }

        else if(autoPathSelection == 6.0) {

            if(pathNumber == 1) {
                returnTrajectory = blue1Trajectory1;
            }
            else if(pathNumber == 2) {
                returnTrajectory = blue1Trajectory1;
            }
            else if(pathNumber == 3) {
                returnTrajectory = blue1Trajectory1;
            }
        }

        return returnTrajectory;

    }

    public static Trajectory returnAutoTrajectory() {

        return returnTrajectory;

    }
}