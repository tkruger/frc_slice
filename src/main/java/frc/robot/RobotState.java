// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class RobotState {

    public double elevatorHeight;
    public double wristAngle;

    /**
     * Creates a new instance of the RobotState data class. Use this 
     * class to store the elevator heights(rotations) and the
     * wrist angles(degrees) for the various elevator and 
     * wrist configuration states.
     * 
     * @param elevatorHeight The elevator height(rotations for now) of the state.
     * @param wristAngle The wrist angle(rotations for now) of the state.
     */
    public RobotState(double elevatorHeight, double wristAngle) {

        this.elevatorHeight = elevatorHeight;
        this.wristAngle = wristAngle;

    }

}
