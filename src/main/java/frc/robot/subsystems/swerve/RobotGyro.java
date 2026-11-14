package frc.robot.subsystems.swerve;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.CalibrationTime;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;

/** Hardware-specific gyro reads. All heading values returned here are CCW positive. */
public class RobotGyro {
  private ADIS16470_IMU adis;
  private ADXRS450_Gyro adxrs;

  public RobotGyro() {
    switch (Constants.GYRO_TYPE) {
      case ADIS16470 -> adis = new ADIS16470_IMU(
          IMUAxis.kZ, IMUAxis.kX, IMUAxis.kY, Constants.GYRO_SPI_PORT, CalibrationTime._1s);
      case ADXRS450 -> adxrs = new ADXRS450_Gyro(Constants.GYRO_SPI_PORT);
      case NOT_CONFIGURED -> DriverStation.reportWarning(
          "Field-centric swerve needs a gyro. Select your actual sensor in Constants.java.", false);
    }
  }

  public boolean isConnected() {
    if (adis != null) {
      return adis.isConnected();
    }
    return adxrs != null && adxrs.isConnected();
  }

  public double getHeadingRadians() {
    double degrees;
    if (adis != null) {
      degrees = adis.getAngle(); // ADIS16470 is already counterclockwise positive.
    } else if (adxrs != null) {
      degrees = -adxrs.getAngle(); // ADXRS450 reports clockwise positive.
    } else {
      return Double.NaN; // Never substitute a fake heading for a missing gyro.
    }
    return Math.toRadians(degrees) * (Constants.GYRO_REVERSED ? -1.0 : 1.0);
  }
}
