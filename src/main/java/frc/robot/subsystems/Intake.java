package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
//import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.StatusFrame;
//import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
//import com.ctre.phoenix.motorcontrol.can.SlotConfiguration;
//import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
//import com.ctre.phoenix.motorcontrol.can.TalonSRXPIDSetConfiguration;
//import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.RobotContainer;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

    private final TalonFX intakeMotor;
    private TalonFXConfiguration config;

    private Faults faults;

    public Intake() {

        intakeMotor = new TalonFX(Constants.intake_MOTOR_PORT);
        config = new TalonFXConfiguration();
        faults = new Faults();

    }

    public void runIntake(boolean intakeMotorForwardToggle, boolean intakeMotorBackward) {

        intakeMotor.configAllSettings(config);
    
        intakeMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 100);

        System.out.println(intakeMotor.getSelectedSensorPosition());
        System.out.println(intakeMotor.getSelectedSensorVelocity());
        System.out.println(intakeMotor.getMotorOutputPercent());
        System.out.println(intakeMotor.getStatorCurrent());

        ErrorCode error = intakeMotor.getLastError();
        ErrorCode faultsError = intakeMotor.getFaults(faults);

        System.out.println(error);
        System.out.println(faultsError);
    
        if(intakeMotorForwardToggle == true) {
            intakeMotor.set(TalonFXControlMode.PercentOutput, -.5);
        }
        else if(intakeMotorBackward == true) {
            intakeMotor.set(TalonFXControlMode.PercentOutput, .5);
        }
        else {
            intakeMotor.set(TalonFXControlMode.PercentOutput, 0.0);
        }

    }

    @Override
    public void periodic() {
    // This method will be called once per scheduler run
    }

}


    
    
