// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * A class to filter the input of a joystick axis.
 */
public class JoyFilter {
    private double lastInput;
    private final double deadzone, smoothing;

    /**
     * @param deadzone input from the joystick that is less than the deadzone will be ignored. -1 to 1
     * @param smoothing a higher value will smooth the input more but also increase input delay. 0 to 1
     */
    public JoyFilter(double deadzone, double smoothing) {
        this.deadzone = deadzone;
        this.smoothing = smoothing;
        
        lastInput = 0;
    }

    public double withDead(double raw) {
        if (raw < deadzone) {
            return 0;
        }else {
            return (raw - deadzone * Math.signum(raw))/(1 - deadzone);
        }
    }

    public double filter(double raw) {
        double signal = smoothing * lastInput + (1 - smoothing) * raw;
        lastInput = signal;
        signal = withDead(signal);
        return signal;
    }
}
