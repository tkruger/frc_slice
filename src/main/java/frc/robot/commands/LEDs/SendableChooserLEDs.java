package frc.robot.commands.LEDs;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.LEDs.LEDMode;

public class SendableChooserLEDs extends CommandBase {
    private final LEDs leds;
    private int m_rainbowFirstPixelHue = 0;
    private int range = 20;

    private LEDMode ledMode;
    private LEDMode lastLEDMode;
    
    public SendableChooserLEDs(LEDs leds) {
      this.leds = leds;

      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(leds);
    }

    @Override
    public void initialize() {
      leds.setAll(Color.kBlack);
    }

    @Override
    public void execute() {

      ledMode = leds.getSelectedMode();

      if(ledMode != lastLEDMode) {

        leds.setAll(Color.kBlack);

      }

      if(ledMode == LEDMode.RAINBOW) {

        for(int i = 0; i < 77; i++) {
          final int hue = (m_rainbowFirstPixelHue + (i * 180 / 77 )) % 180;
          leds.setLEDhsv(i, hue, 255, 128);
        }

        m_rainbowFirstPixelHue += 1.5;
        m_rainbowFirstPixelHue %= 180;

        leds.ledBuffer();

      }
      else {

        for(int i = 0; i < 77; i++) {
            final int hue = (((m_rainbowFirstPixelHue + (i * range / 77 )) % range) + 140) % 180;
            leds.setLEDhsv(i, hue, 255, 128);
        }

        m_rainbowFirstPixelHue += 0.5;
        m_rainbowFirstPixelHue %= range;

        leds.ledBuffer();

      }

      lastLEDMode = leds.getSelectedMode();

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      leds.setAll(Color.kBlack);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }

    @Override
    public boolean runsWhenDisabled() {
      return true;
    }
}
