package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import frc.robot.Robot;

public class Indexer {

    //Declare indexer motor
    private final PWMSparkMax indexMotor = new PWMSparkMax(7);

    //Instantiate Classes
    private Robot joysticks = new Robot();

    public void runIndexer() {

        //Run indexer when designated button is pressed
        if(joysticks.leftJoystick.getRawButton(1) == true) {
            indexMotor.set(0.6);
        }
        
    }
    
}