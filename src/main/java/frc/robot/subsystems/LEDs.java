// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

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
    leds = new AddressableLED(Constants.cLEDs.port);
    buffer = new AddressableLEDBuffer(Constants.cLEDs.count);

    leds.setLength(Constants.cLEDs.count);
    leds.start();
  }

  /**
   * Sets all LEDs along the robot to the same color
   * @param color the color the leds will display
   */
  public void setAll(Color color) {
    Color newColor = new Color(color.red, color.blue, color.green);
    for (int i = 0; i < Constants.cLEDs.count; i++) {
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
    for (int i = 0; i < Constants.cLEDs.count; i++) {
      buffer.setRGB(i, r, g, b);
    }
    leds.setData(buffer);
  }

  public void setAllHSV(int h, int s, int v) {
    for (int i = 0; i < Constants.cLEDs.count; i++) {
      buffer.setHSV(i, h, s, v);
    }
    leds.setData(buffer);
  }

  public void setLEDhsv(int index, int h, int s, int v) {
    buffer.setHSV(index, h, s, v);
  }

  public void ledBuffer() {
    leds.setData(buffer);
  }

  public void setSegmenthsv(int[] indices, int h, int s, int v) {
    for (int i = indices[0]; i < indices[1]; i++) {
      buffer.setHSV(i, h, s, v);
    }
    leds.setData(buffer);
  }

  public void balanceLedMode(double angle) {
    // angles range between 0 degrees and 15 degrees

    double excludedLeft = (Constants.cLEDs.left[1] - Constants.cLEDs.left[0]) * (1 - (angle * .06667)) + Constants.cLEDs.left[0];
    double excludedRight = Constants.cLEDs.right[1] - (Constants.cLEDs.right[1] - Constants.cLEDs.right[0]) * (1 - (angle * .06667));

    // sets all leds to black
    for (int i = 0; i < Constants.cLEDs.count; i++) {
      buffer.setRGB(i, 0, 0, 0);
    }

    for (int i = Constants.cLEDs.left[0]; i < excludedLeft; i++) {
      buffer.setRGB(i, 255, 140, 60);
    }

    for (int i = Constants.cLEDs.right[1]; i > excludedRight; i--) {
      buffer.setRGB(i, 255, 140, 60);
    }

    leds.setData(buffer);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
