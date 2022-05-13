package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import frc.robot.Robot;

public class Indexer {

    //Declare indexer motor
    private static final PWMSparkMax indexMotor = new PWMSparkMax(7);

    public static void runIndexer() {

        //Run indexer when designated button is pressed
        if(Robot.leftJoystick.getRawButton(1) == true) {
            indexMotor.set(0.6);
        }
        
    }
    
}