package frc.robot.Subsystems;

import frc.robot.Robot;
import com.revrobotics.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Indexer {

    //Declare indexer motor
    private final CANSparkMax indexMotor = new CANSparkMax(7, MotorType.kBrushless);

    //Instantiate Classes
    private Robot joysticks = new Robot();

    public void runIndexer() {

        //Run indexer when designated button is pressed
        if(joysticks.leftJoystick.getRawButton(1) == true) {
            indexMotor.set(0.6);
        }
        
    }
    
}