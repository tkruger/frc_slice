package frc.robot.Subsystems;

//import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;

public class GyroCode {
    
  // Creates an ADXRS450_Gyro object on the onboard SPI port
  ADXRS450_Gyro gyro = new ADXRS450_Gyro();

  public double robotAngle = 0;

  public void gyroInit() {
    //Calibrates the Gyro before movement
    gyro.calibrate();
  }

  public double gyroUpdate() {
    robotAngle = gyro.getAngle();
    return robotAngle;
  }
}
