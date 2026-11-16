package frc.robot.subsystems.swerve;

import com.reduxrobotics.sensors.canandgyro.Canandgyro;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.CalibrationTime;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;

/** Hardware-specific gyro reads. All heading values returned here are CCW positive. */
public class RobotGyro {
  private Canandgyro canandgyro;
  private ADIS16470_IMU adis;
  private ADXRS450_Gyro adxrs;

  public RobotGyro() {
    switch (Constants.GYRO_TYPE) {
      case CANANDGYRO -> canandgyro = new Canandgyro(Constants.GYRO_CAN_ID);
      case ADIS16470 -> adis = new ADIS16470_IMU(
          IMUAxis.kZ, IMUAxis.kX, IMUAxis.kY, Constants.GYRO_SPI_PORT, CalibrationTime._1s);
      case ADXRS450 -> adxrs = new ADXRS450_Gyro(Constants.GYRO_SPI_PORT);
      case NOT_CONFIGURED -> DriverStation.reportWarning(
          "Field-centric swerve needs a gyro. Select your actual sensor in Constants.java.", false);
    }
  }

  public boolean isConnected() {
    if (canandgyro != null) {
      return canandgyro.isConnected();
    }
    if (adis != null) {
      return adis.isConnected();
    }
    return adxrs != null && adxrs.isConnected();
  }

  public double getHeadingRadians() {
    double radians;
    if (canandgyro != null) {
      // Do not drive using an initial/default heading or a calibration in progress.
      if (!canandgyro.isConnected() || canandgyro.isCalibrating()
          || !canandgyro.getYawFrame().hasData()) {
        return Double.NaN;
      }
      // CANandgyro reports CCW-positive rotations, not degrees.
      radians = canandgyro.getYaw() * 2.0 * Math.PI;
    } else if (adis != null) {
      radians = Math.toRadians(adis.getAngle()); // Already counterclockwise positive.
    } else if (adxrs != null) {
      radians = Math.toRadians(-adxrs.getAngle()); // ADXRS450 reports clockwise positive.
    } else {
      return Double.NaN; // Never substitute a fake heading for a missing gyro.
    }
    return radians * (Constants.GYRO_REVERSED ? -1.0 : 1.0);
  }
}
