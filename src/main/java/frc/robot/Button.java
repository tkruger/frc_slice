package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class Button {

    //Define Joysticks
    public static Joystick leftJoystick = new Joystick(Constants.Button_LEFT_JOYSTICK_PORT);
    public static Joystick rightJoystick = new Joystick(Constants.Button_RIGHT_JOYSTICK_PORT);

    //Unassigned Right Joystick Buttons
    public static JoystickButton rightButton1 = new JoystickButton(rightJoystick, 1); //Right Top 1
    public static JoystickButton rightButton2 = new JoystickButton(rightJoystick, 2); //Right Top 2
    public static JoystickButton rightButton3 = new JoystickButton(rightJoystick, 3); //Right Top 3
    public static JoystickButton rightButton4 = new JoystickButton(rightJoystick, 4); //Right Top 4
    public static JoystickButton rightButton5 = new JoystickButton(rightJoystick, 5); //Right Top 5
    public static JoystickButton rightButton6 = new JoystickButton(rightJoystick, 6); //Right Top 6
    public static JoystickButton rightButton7 = new JoystickButton(rightJoystick, 7); //Right Bottom 7
    public static JoystickButton rightButton8 = new JoystickButton(rightJoystick, 8); //Right Bottom 8
    public static JoystickButton rightButton9 = new JoystickButton(rightJoystick, 9); //Right Bottom 9
    public static JoystickButton rightButton10 = new JoystickButton(rightJoystick, 10); //Right Bottom 9
    public static JoystickButton rightButton11 = new JoystickButton(rightJoystick, 11); //Right Bottom 11
    public static JoystickButton rightButton12 = new JoystickButton(rightJoystick, 12); //Right Bottom 12

    //Unassigned Left Joystick Buttons
    public static JoystickButton leftButton1 = new JoystickButton(leftJoystick, 1); //Left Top 1
    public static JoystickButton leftButton2 = new JoystickButton(leftJoystick, 2); //Left Top 2
    public static JoystickButton leftButton3 = new JoystickButton(leftJoystick, 3); //Left Top 3
    public static JoystickButton leftButton4 = new JoystickButton(leftJoystick, 4); //Left Top 4
    public static JoystickButton leftButton5 = new JoystickButton(leftJoystick, 5); //Left Top 5
    public static JoystickButton leftButton6 = new JoystickButton(leftJoystick, 6); //Left Top 6
    public static JoystickButton leftButton7 = new JoystickButton(leftJoystick, 7); //Left Bottom 7
    public static JoystickButton leftButton8 = new JoystickButton(leftJoystick, 8); //Left Bottom 8
    public static JoystickButton leftButton9 = new JoystickButton(leftJoystick, 9); //Left Bottom 9
    public static JoystickButton leftButton10 = new JoystickButton(leftJoystick, 10); //Left Bottom 10
    public static JoystickButton leftButton11 = new JoystickButton(leftJoystick, 11); //Left Bottom 11
    public static JoystickButton leftButton12 = new JoystickButton(leftJoystick, 12); //Left Bottom 12
    
}
