package frc.robot.Subsystems;

//import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;

public class GyroCode {
    
  // Creates an ADXRS450_Gyro object on the onboard SPI port
  static ADXRS450_Gyro gyro = new ADXRS450_Gyro();

  public static void gyroInit() {
    //Calibrates the Gyro before movement
    gyro.calibrate();
  }
}
