package frc.robot;

import edu.wpi.first.wpilibj.SPI;

/** Hardware settings to fill in before driving. */
public final class Constants {
  private Constants() {}

  public static final int PS4_PORT = 0;
  public static final double STICK_DEADBAND = 0.08;

  // All arrays use this order: front left, front right, back left, back right.
  // Rear IDs are inferred from your front module numbering. Verify them.
  public static final int[] DRIVE_CAN_IDS = {1, 4, 7, 10};
  public static final int[] STEER_CAN_IDS = {3, 6, 9, 12};
  public static final String[] MODULE_NAMES = {"FL", "FR", "BL", "BR"};

  // TODO: Measure between the steering pivot centers, in meters.
  public static final double WHEELBASE_METERS = 0.0; // Front to back.
  public static final double TRACKWIDTH_METERS = 0.0; // Left to right.

  // TODO: Motor rotations for ONE full rotation of the module, NOT the wheel.
  // Example only: a 12.8:1 steering reduction would use 12.8.
  // "1650" is the NEO part number, not a gear ratio.
  public static final double STEER_GEAR_RATIO = 0.0;

  // TODO: Actual wheel diameter and drive motor rotations per wheel rotation.
  // Both are needed to measure odometry in meters.
  public static final double WHEEL_DIAMETER_METERS = 0.0;
  public static final double DRIVE_GEAR_RATIO = 0.0;

  // Redux Robotics Boron CANandgyro, connected over CAN.
  public enum GyroType { NOT_CONFIGURED, CANANDGYRO, ADIS16470, ADXRS450 }
  public static final GyroType GYRO_TYPE = GyroType.CANANDGYRO;

  public static final int GYRO_CAN_ID = 0;
  public static final SPI.Port GYRO_SPI_PORT = SPI.Port.kOnboardCS0;
  // With the gyro's +Z axis up, counterclockwise rotation should increase heading.
  public static final boolean GYRO_REVERSED = false;

  // Check each module on blocks. Positive drive must roll the robot forward
  // when zeroed. Positive steer must turn the module counterclockwise from above.
  public static final boolean[] DRIVE_INVERTED = {false, false, false, false};
  public static final boolean[] STEER_INVERTED = {false, false, false, false};
  // Independently check the sensor: counterclockwise module motion must INCREASE
  // the dashboard angle. Use -1.0 for any encoder that reports the opposite sign.
  public static final double[] STEER_ENCODER_SIGN = {1.0, 1.0, 1.0, 1.0};
  // Positive distance must mean rolling FORWARD in the module's zero direction.
  public static final double[] DRIVE_ENCODER_SIGN = {1.0, 1.0, 1.0, 1.0};

  // Conservative starting outputs; tune after checking directions on blocks.
  public static final double MAX_DRIVE_OUTPUT = 0.20;
  public static final double MAX_STEER_OUTPUT = 0.20;
  public static final double STEER_KP = 0.50; // Motor output per radian of error.
  public static final int DRIVE_CURRENT_LIMIT_AMPS = 40;
  public static final int STEER_CURRENT_LIMIT_AMPS = 20;

  public static boolean hardwareConfigured() {
    return positive(WHEELBASE_METERS)
        && positive(TRACKWIDTH_METERS)
        && positive(STEER_GEAR_RATIO)
        && positive(WHEEL_DIAMETER_METERS)
        && positive(DRIVE_GEAR_RATIO);
  }

  private static boolean positive(double value) {
    return Double.isFinite(value) && value > 0.0;
  }
}
