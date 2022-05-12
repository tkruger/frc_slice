package frc.robot;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

public class Indexer {
    private static final PWMSparkMax indexMotor = new PWMSparkMax(7);

    public static void runIndexer() {

        if(Robot.leftJoystick.getRawButton(1) == true) {
            indexMotor.set(.6);
        }
        
    }
    
}