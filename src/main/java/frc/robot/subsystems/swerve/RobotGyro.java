package frc.robot.subsystems.swerve;

import com.reduxrobotics.sensors.canandgyro.Canandgyro;
import frc.robot.Constants;

public class RobotGyro {
  private final Canandgyro canandgyro;

  public RobotGyro() {
    canandgyro = new Canandgyro(Constants.GYRO_CAN_ID);
  }

  public boolean isConnected() {
    return canandgyro.isConnected();
  }

  public double getHeadingRadians() {
    if (!canandgyro.isConnected() || canandgyro.isCalibrating()
        || !canandgyro.getYawFrame().hasData()) {
      return Double.NaN;
    }
    double radians = canandgyro.getYaw() * 2.0 * Math.PI;
    return radians * (Constants.GYRO_REVERSED ? -1.0 : 1.0);
  }
}
