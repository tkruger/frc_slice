package frc.robot;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

public class Indexer {

    //Declare indexer motor
    private static final PWMSparkMax indexMotor = new PWMSparkMax(7);

    public static void runIndexer() {

        //run indexer when designated button is pressed
        if(Robot.leftJoystick.getRawButton(1) == true) {
            indexMotor.set(.6);
        }

    }
    
}