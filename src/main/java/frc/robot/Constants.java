package frc.robot;

/** Hardware settings to fill in before driving. */
public final class Constants {
  private Constants() {}

  public static final int PS4_PORT = 0;
  public static final double STICK_DEADBAND = 0.08;

  // All arrays use this order: front left, front right, back left, back right.
  // Rear IDs are inferred from your front module numbering. Verify them.
  public static final int[] DRIVE_CAN_IDS = {1, 4, 7, 10};
  public static final int[] STEER_CAN_IDS = {3, 6, 9, 12};
  // CTRE absolute magnetic CANcoders running firmware 26.1.1 (Phoenix 6).
  // Phoenix Tuner X names, in the same FL, FR, BL, BR order as the other arrays.
  public static final int[] CANCODER_CAN_IDS = {
      2,  // Frontleft CANcoder
      5,  // Frontright CANcoder
      11, // Rearleft CANcoder
      8   // Rearright CANcoder
  };
  // CANcoder absolute reading (rotations) with each module physically facing forward.
  // Use 0 only if forward has already been calibrated to zero in Phoenix Tuner.
  // These offsets are applied in software; existing device configuration is preserved.
  public static final double[] CANCODER_FORWARD_ROTATIONS = {
      0, 0, 0, 0};
  public static final double CANCODER_MAX_AGE_SECONDS = 0.10;
  public static final String[] MODULE_NAMES = {"FL", "FR", "BL", "BR"};

  
  public static final double WHEELBASE_METERS = 0.377825; // Front to back.
  public static final double TRACKWIDTH_METERS = 0.377825; // Left to right.


  // Both are needed to measure odometry in meters.
  public static final double WHEEL_DIAMETER_METERS = 0.1016;
  public static final double DRIVE_GEAR_RATIO = 6.12;

  // Redux Robotics Boron CANandgyro, connected over CAN.
  public static final int GYRO_CAN_ID = 0;
  // With the gyro's +Z axis up, counterclockwise rotation should increase heading.
  public static final boolean GYRO_REVERSED = false;

  // Check each module on blocks. Positive drive must roll the robot forward
  // when zeroed. Positive steer must turn the module counterclockwise from above.
  public static final boolean[] DRIVE_INVERTED = {false, true, false, true};
  public static final boolean[] STEER_INVERTED = {true, true, true, true};
  // CANcoder direction: counterclockwise module motion must INCREASE
  // the dashboard angle. Use -1.0 for any encoder that reports the opposite sign.
  public static final double[] STEER_ENCODER_SIGN = {1.0, 1.0, 1.0, 1.0};
  // Positive distance must mean rolling FORWARD in the module's zero direction.
  public static final double[] DRIVE_ENCODER_SIGN = {1.0, 1.0, 1.0, 1.0};

  // Conservative starting outputs; tune after checking directions on blocks.
  public static final double MAX_DRIVE_OUTPUT = 0.20;
  public static final double MAX_STEER_OUTPUT = 0.20;
  public static final double STEER_KP = 0.2; // Motor output per radian of error.
  // Accept up to +/-0.12 CANcoder rotations (43.2 degrees) from calibrated forward.
  public static final double STEER_ALIGNMENT_TOLERANCE_RADIANS = 0.1 *  Math.PI;
  public static final int DRIVE_CURRENT_LIMIT_AMPS = 40;
  public static final int STEER_CURRENT_LIMIT_AMPS = 20;

  public static boolean hardwareConfigured() {
    return positive(WHEELBASE_METERS)
        && positive(TRACKWIDTH_METERS)
        && positive(WHEEL_DIAMETER_METERS)
        && positive(DRIVE_GEAR_RATIO);
  }

  private static boolean positive(double value) {
    return Double.isFinite(value) && value > 0.0;
  }
}
