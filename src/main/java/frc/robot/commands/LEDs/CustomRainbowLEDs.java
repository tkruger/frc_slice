package frc.robot.commands.LEDs;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDs;

public class CustomRainbowLEDs extends CommandBase {
    private final LEDs leds;
    private int offset;
    private final int m_centerHue, m_range;
    
    public CustomRainbowLEDs(LEDs leds, int hue, int range) {
      this.leds = leds;
      m_centerHue = hue;
      m_range = range;
      offset = 0;

      // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(leds);
    }

    @Override
    public void initialize() {
      leds.setAll(Color.kBlack);
    }

    @Override
    public void execute() {
        for(int i = 0; i < 77; i++) {
            int shift = m_range * (int) Math.sin((i+offset) * 2.0 * Math.PI / 77.0);
            int hue = (m_centerHue + shift) % 180;
            leds.setLEDhsv(i, hue, 255, 255);
        }

        offset++;
        offset %= 77;

        leds.ledBuffer();
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
}
