package frc.robot.auto;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.*;

public class Paths {

    static Trajectory blueLeftTrajectory1;
    static Path blueLeftTrajectoryPath1;
    static String blueLeftPath1JSON;

    static Trajectory blueLeftTrajectory2;
    static Path blueLeftTrajectoryPath2;
    static String blueLeftPath2JSON;

    static Trajectory blueLeftTrajectory3;
    static Path blueLeftTrajectoryPath3;
    static String blueLeftPath3JSON;

    static Trajectory returnTrajectory;

    private static ShuffleboardTab smartDashboardTab;
    static Double autoPathSelection;
    private static SimpleWidget autoPathWidget; 

    public static void createAutoPathWidget() {

        smartDashboardTab = Shuffleboard.getTab("SmartDashboard");
        autoPathWidget = smartDashboardTab.add("Auto Selector", 1).withWidget(BuiltInWidgets.kNumberSlider);
        autoPathSelection = autoPathWidget.getEntry().getDouble(1.0);

    }

    public static Trajectory getAutoPath(int trajectoryNumber) {
        //Path string variable declarations
        blueLeftPath1JSON = "output/Blue Left Path 1.wpilib.json";
        blueLeftPath2JSON = "output/Blue Left Path 2.wpilib.json";
        blueLeftPath3JSON = "output/Blue Left Path 3.wpilib.json";
        /*blueLeftPath1JSON = "pathplanner/generatedJSON/Blue Left Path 1.wpilib.json";
        blueLeftPath2JSON = "pathplanner/generatedJSON/Blue Left Path 2.wpilib.json";
        blueLeftPath3JSON = "pathplanner/generatedJSON/Blue Left Path 3.wpilib.json";*/

        //Trajectory and path object declarations
        blueLeftTrajectory1 = new Trajectory();
        blueLeftTrajectoryPath1 = Filesystem.getDeployDirectory().toPath().resolve(blueLeftPath1JSON);
        blueLeftTrajectory2 = new Trajectory();
        blueLeftTrajectoryPath2 = Filesystem.getDeployDirectory().toPath().resolve(blueLeftPath2JSON);
        blueLeftTrajectory3 = new Trajectory();
        blueLeftTrajectoryPath3 = Filesystem.getDeployDirectory().toPath().resolve(blueLeftPath3JSON);

        //Path to trajecectory conversions
        try {
            blueLeftTrajectory1 = TrajectoryUtil.fromPathweaverJson(blueLeftTrajectoryPath1);
        } 
        catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + blueLeftPath1JSON, ex.getStackTrace());
        }
        try {
            blueLeftTrajectory2 = TrajectoryUtil.fromPathweaverJson(blueLeftTrajectoryPath2);
        }
        catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + blueLeftPath2JSON, ex.getStackTrace());
        }
        try {
            blueLeftTrajectory3 = TrajectoryUtil.fromPathweaverJson(blueLeftTrajectoryPath3);
        }
        catch(IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + blueLeftPath3JSON, ex.getStackTrace());
        }

        /*Trajectory selection for autonomous
        (Many of these return statements use blueLeftTrajectory1 as a placeholder for now until more trajectory and path objects are created)*/
        if(autoPathSelection == 1.0) {

            if(trajectoryNumber == 1) {
                returnTrajectory = blueLeftTrajectory1;
            }
            else if(trajectoryNumber == 2) {
                returnTrajectory = blueLeftTrajectory2;
            }
            else if(trajectoryNumber == 3) {
                returnTrajectory = blueLeftTrajectory3;
            }
        }

        else if(autoPathSelection == 2.0) {

            if(trajectoryNumber == 1) {
                returnTrajectory = blueLeftTrajectory1;
            }
            else if(trajectoryNumber == 2) {
                returnTrajectory = blueLeftTrajectory1;
            }
            else if(trajectoryNumber == 3)
                returnTrajectory = blueLeftTrajectory1;

        }

        else if(autoPathSelection == 3.0) {

            if(trajectoryNumber == 1) {
                returnTrajectory = blueLeftTrajectory1;
            }
            else if(trajectoryNumber == 2) {
                returnTrajectory = blueLeftTrajectory1;
            }
            else if(trajectoryNumber == 3) {
                returnTrajectory = blueLeftTrajectory1;
            }

        }

        else if(autoPathSelection == 4.0) {

            if(trajectoryNumber == 1) {
                returnTrajectory = blueLeftTrajectory1;
            }
            else if(trajectoryNumber == 2) {
                returnTrajectory = blueLeftTrajectory1;
            }
            else if(trajectoryNumber == 3) {
                returnTrajectory = blueLeftTrajectory1;
            }
        }

        else if(autoPathSelection == 5.0) {

            if(trajectoryNumber == 1) {
                returnTrajectory = blueLeftTrajectory1;
            }
            else if(trajectoryNumber == 2) {
                returnTrajectory = blueLeftTrajectory1;
            }
            else if(trajectoryNumber == 3) {
                returnTrajectory = blueLeftTrajectory1;
            }
        }

        else if(autoPathSelection == 6.0) {

            if(trajectoryNumber == 1) {
                returnTrajectory = blueLeftTrajectory1;
            }
            else if(trajectoryNumber == 2) {
                returnTrajectory = blueLeftTrajectory1;
            }
            else if(trajectoryNumber == 3) {
                returnTrajectory = blueLeftTrajectory1;
            }
        }

        return returnTrajectory;

    }

    public static Trajectory returnAutoTrajectory() {

        return returnTrajectory;

    }

    public static Trajectory returnPlaceholderTrajectory() {
        return new Trajectory();
    }
}