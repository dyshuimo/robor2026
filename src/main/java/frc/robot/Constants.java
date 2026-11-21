package frc.robot;

public final class Constants {
  private Constants() {}

  public static final int PS4_PORT = 0;
  public static final double STICK_DEADBAND = 0.08;

  public static final int[] DRIVE_CAN_IDS = {1, 4, 7, 10};
  public static final int[] STEER_CAN_IDS = {3, 6, 9, 12};

  public static final int[] CANCODER_CAN_IDS = {2, 5, 11, 8};


  public static final double[] CANCODER_FORWARD_ROTATIONS = {
      0, 0, 0, 0};
  public static final double CANCODER_MAX_AGE_SECONDS = 0.10;
  public static final String[] MODULE_NAMES = {"FL", "FR", "BL", "BR"};

  
  public static final double WHEELBASE_METERS = 0.377825;
  public static final double TRACKWIDTH_METERS = 0.377825;


  public static final double WHEEL_DIAMETER_METERS = 0.1016;
  public static final double DRIVE_GEAR_RATIO = 6.12;

  public static final int GYRO_CAN_ID = 0;
  public static final boolean GYRO_REVERSED = false;


  public static final boolean[] DRIVE_INVERTED = {false, true, false, true};
  public static final boolean[] STEER_INVERTED = {true, true, true, true};
  public static final double[] STEER_ENCODER_SIGN = {1.0, 1.0, 1.0, 1.0};
  public static final double[] DRIVE_ENCODER_SIGN = {1.0, 1.0, 1.0, 1.0};

  public static final double MAX_DRIVE_OUTPUT = 0.20;
  public static final double MAX_STEER_OUTPUT = 0.20;
  public static final double STEER_KP = 0.2;

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
