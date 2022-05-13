package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import frc.robot.Robot;

public class Climber {

    //Declare Climb Motors
    private static final PWMSparkMax leftClimbMotor = new PWMSparkMax(5);
    private static final PWMSparkMax rightClimbMotor = new PWMSparkMax(6);
    private static final MotorControllerGroup climbMotors = new MotorControllerGroup(leftClimbMotor, rightClimbMotor);

    public static void climbArms() {

        //Extend Both Climb Arms
        if(Robot.leftJoystick.getRawButton(2) == true) {
            climbMotors.set(0.6);
        }
        
        //Extend Left Climb Arm
        if(Robot.leftJoystick.getRawButton(4) == true) {
            leftClimbMotor.set(0.6);
        }

        //Extend Right Climb Arm
        if(Robot.leftJoystick.getRawButton(3) == true) {
            rightClimbMotor.set(0.6);
        }

        //Retract Both Climb Arms
        if(Robot.rightJoystick.getRawButton(2) == true) {
            climbMotors.set(-0.6);
        }
        
        //Retract Left Climb Arm
        if(Robot.rightJoystick.getRawButton(4) == true) {
            leftClimbMotor.set(-0.6);
        }

        //Retract Right Climb Arm
        if(Robot.rightJoystick.getRawButton(3) == true) {
            rightClimbMotor.set(-0.6);
        }

    }
}