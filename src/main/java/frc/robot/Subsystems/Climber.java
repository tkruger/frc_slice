package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import com.revrobotics.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Robot;

public class Climber {

    //Declare Climb Motors
    private final CANSparkMax leftClimbMotor = new CANSparkMax(5, MotorType.kBrushless);
    private final CANSparkMax rightClimbMotor = new CANSparkMax(6, MotorType.kBrushless);
    private final MotorControllerGroup climbMotors = new MotorControllerGroup(leftClimbMotor, rightClimbMotor);

    //Instantiate Classes
    private Robot joysticks = new Robot();

    public void climbArms() {

        //Extend Both Climb Arms
        if(joysticks.leftJoystick.getRawButton(2) == true) {
            climbMotors.set(0.6);
        }
        
        //Extend Left Climb Arm
        if(joysticks.leftJoystick.getRawButton(4) == true) {
            leftClimbMotor.set(0.6);
        }

        //Extend Right Climb Arm
        if(joysticks.leftJoystick.getRawButton(3) == true) {
            rightClimbMotor.set(0.6);
        }

        //Retract Both Climb Arms
        if(joysticks.rightJoystick.getRawButton(2) == true) {
            climbMotors.set(-0.6);
        }
        
        //Retract Left Climb Arm
        if(joysticks.rightJoystick.getRawButton(4) == true) {
            leftClimbMotor.set(-0.6);
        }

        //Retract Right Climb Arm
        if(joysticks.rightJoystick.getRawButton(3) == true) {
            rightClimbMotor.set(-0.6);
        }

    }
}