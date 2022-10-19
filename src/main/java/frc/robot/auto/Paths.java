package frc.robot.auto;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class Paths {

    static Trajectory blue1Trajectory1;
    static Path blue1TrajectoryPath1;
    static String blue1Trajectory1JSON;

    static Trajectory blue1Trajectory2;
    static Path blue1TrajectoryPath2;
    static String blue1Trajectory2JSON;

    static String autoPathSelection;
    static Trajectory returnTrajectory;

    public static Trajectory getAutoPath(int pathNumber) {

        ShuffleboardTab smartDashboardTab = Shuffleboard.getTab("SmartDashboard");
        autoPathSelection = smartDashboardTab.add("Auto Path", "").getEntry().getString("");

        //Path string variable declarations
        blue1Trajectory1JSON = "output/Blue 1 Path 1.wpilib.json";
        blue1Trajectory2JSON = "output/Blue 1 Path 2.wpilib.json";

        //Trajectory object declarations
        blue1Trajectory1 = new Trajectory();
        blue1TrajectoryPath1 = Filesystem.getDeployDirectory().toPath().resolve(blue1Trajectory1JSON);
        blue1Trajectory2 = new Trajectory();
        blue1TrajectoryPath2 = Filesystem.getDeployDirectory().toPath().resolve(blue1Trajectory2JSON);

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

        /*Trajectory selection for autonomous
        (These return statements all use blue1Trajectory1 as a placeholder for now until more trajectory objects are created)*/
        if(autoPathSelection == "Blue 1 Auto") {

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

        else if(autoPathSelection == "Blue 2 Auto") {

            if(pathNumber == 1) {
                returnTrajectory = blue1Trajectory1;
            }
            else if(pathNumber == 2) {
                returnTrajectory = blue1Trajectory1;
            }
            else if(pathNumber == 3)
                returnTrajectory = blue1Trajectory1;

        }

        else if(autoPathSelection == "Blue 3 Auto") {

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

        else if(autoPathSelection == "Red 1 Auto") {

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

        else if(autoPathSelection == "Red 2 Auto") {

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

        else if(autoPathSelection == "Red 3 Auto") {

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
}