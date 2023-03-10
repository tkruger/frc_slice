package frc.robot.subsystems;

import edu.wpi.first.wpilibj.I2C;
//import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
//import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;

//import java.util.Map;

import com.revrobotics.ColorMatch;

public class ColorSensor extends SubsystemBase {
    private final I2C.Port i2cPort = I2C.Port.kOnboard;
    private final ColorSensorV3 m_colorSensor;
    private final ColorMatch m_colorMatcher;

    private final Color kCube, kCone;

    private int proximity;

    private Color detectedColor;

    private ColorMatchResult match;

    //private SimpleWidget colorWidget;

    public ColorSensor() {
        m_colorSensor = new ColorSensorV3(i2cPort);
        m_colorMatcher = new ColorMatch();

        kCube = new Color(0.143, 0.427, 0.429);
        kCone = new Color(0.361, 0.524, 0.113);

        m_colorMatcher.addColorMatch(kCube);
        m_colorMatcher.addColorMatch(kCone);

        //colorWidget = Shuffleboard.getTab("Driver Tab").add("Game Piece Color", false);
        //colorWidget.withProperties(Map.of("colorWhenFalse", "black"));
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        detectedColor = m_colorSensor.getColor();

        SmartDashboard.putNumber("Red", detectedColor.red);
        SmartDashboard.putNumber("Green", detectedColor.green);
        SmartDashboard.putNumber("Blue", detectedColor.blue);

        proximity = m_colorSensor.getProximity();
        SmartDashboard.putNumber("Proximity", proximity);

        match = m_colorMatcher.matchClosestColor(detectedColor);

        /*if(getGamePiece() == 1) {
            colorWidget.withProperties(Map.of("colorWhenTrue", kCube));
            colorWidget.getEntry().setBoolean(true);
        }else if(getGamePiece() == 2) {
            colorWidget.withProperties(Map.of("colorWhenTrue", kCone));
            colorWidget.getEntry().setBoolean(true);
        }else {
            colorWidget.getEntry().setBoolean(false);
        }*/
    }

    public Color getColor() {
        return detectedColor;
    }

    /** Returns 0 for none, 1 for a cube, or 2 for a cone */
    public int getGamePiece() {
        if (proximity > 500) {
            return 0;
        }

        if (match.color == kCube) {
            return 1;
        } else if (match.color == kCone) {
            return 2;
        } else {
            return 0;
        }
    }
}
