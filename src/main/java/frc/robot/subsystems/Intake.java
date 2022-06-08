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

    TalonFX intakeMotor;
    TalonFXConfiguration config;

    Faults faults;

    public Intake() {

        final TalonFX intakeMotor = new TalonFX(Constants.INTAKE_INTAKE_MOTOR);
        TalonFXConfiguration config = new TalonFXConfiguration();

        Faults faults = new Faults();

    }

    public void runIntake(boolean intakeMotorForwardToggle, boolean intakeMotorBackwardToggle) {

        config.supplyCurrLimit.enable = true;
        config.supplyCurrLimit.triggerThresholdCurrent = 40;
        config.supplyCurrLimit.triggerThresholdTime = 1.5;
        config.supplyCurrLimit.currentLimit = 30;

        intakeMotor.configAllSettings(config);
    
        intakeMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10);

        System.out.println(intakeMotor.getSelectedSensorPosition());
        System.out.println(intakeMotor.getSelectedSensorVelocity());
        System.out.println(intakeMotor.getMotorOutputPercent());
        System.out.println(intakeMotor.getStatorCurrent());

        ErrorCode error = intakeMotor.getLastError();
        ErrorCode faultsError = intakeMotor.getFaults(faults);

        System.out.println(error);
        System.out.println(faultsError);
    
        if(intakeMotorForwardToggle == true) {
            intakeMotor.set(TalonFXControlMode.PercentOutput, 1);
        }
        else if(intakeMotorForwardToggle == false) {
            intakeMotor.set(TalonFXControlMode.PercentOutput, 0);
        }

        if(intakeMotorBackwardToggle == true) {
            intakeMotor.set(TalonFXControlMode.PercentOutput, -1);
        }
        else if(intakeMotorBackwardToggle == false) {
            intakeMotor.set(TalonFXControlMode.PercentOutput, 0);
        }

    }

    @Override
    public void periodic() {
    // This method will be called once per scheduler run
    }

}


    
    
