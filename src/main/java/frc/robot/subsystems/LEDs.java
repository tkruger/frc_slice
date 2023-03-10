// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDs extends SubsystemBase {
  AddressableLED leds;
  AddressableLEDBuffer buffer;
  
  /** Creates a new LEDs. */
  public LEDs() {
    leds = new AddressableLED(Constants.LEDs.port);
    buffer = new AddressableLEDBuffer(Constants.LEDs.count);

    leds.setLength(Constants.LEDs.count);
    leds.start();
  }

  /**
   * Sets all LEDs along the robot to the same color
   * @param color the color the leds will display
   */
  public void setAll(Color color) {
    Color newColor = new Color(color.red, color.blue, color.green);
    for (int i = 0; i < Constants.LEDs.count; i++) {
      buffer.setLED(i, newColor);
    }
    leds.setData(buffer);
  }

  /**
   * Sets all LEDs along the robot to the same color
   * @param r red part of the color the leds will display
   * @param g green part of the color the leds will display
   * @param b blue part of the color the leds will display
   */
  public void setAll(int r, int g, int b) {
    for (int i = 0; i < Constants.LEDs.count; i++) {
      buffer.setRGB(i, r, g, b);
    }
    leds.setData(buffer);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
