package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/** Field-centric swerve: measured odometry, open-loop drive, proportional steering. */
public class SwerveDrive {
  private final SwerveModule[] modules = new SwerveModule[4];
  private final RobotGyro gyro = new RobotGyro();
  private final SwervePoseTracker poseTracker;
  private final Field2d field = new Field2d();
  private Rotation2d rawHeading = new Rotation2d();
  private final SwerveModulePosition[] positions = new SwerveModulePosition[4];
  private boolean sensorsReady;

  public SwerveDrive() {
    for (int i = 0; i < modules.length; i++) {
      modules[i] = new SwerveModule(
          Constants.MODULE_NAMES[i], Constants.DRIVE_CAN_IDS[i], Constants.STEER_CAN_IDS[i],
          Constants.DRIVE_INVERTED[i], Constants.STEER_INVERTED[i], Constants.STEER_ENCODER_SIGN[i],
          Constants.DRIVE_ENCODER_SIGN[i]);
    }
    poseTracker = Constants.hardwareConfigured()
        ? new SwervePoseTracker(Constants.WHEELBASE_METERS, Constants.TRACKWIDTH_METERS) : null;
    SmartDashboard.putData("Swerve/Field", field);
    if (!Constants.hardwareConfigured()) {
      DriverStation.reportWarning(
          "Swerve: set wheelbase, trackwidth, wheel diameter, and both gear ratios in Constants.java.",
          false);
    }
  }

  /** Inputs -1 to +1: FIELD forward, FIELD left, and counterclockwise spin. */
  public void drive(double forward, double left, double turn) {
    if (!isReady()) {
      stop();
      return;
    }
    WheelRequest[] requests = calculateFieldRelativeWheelRequests(
        forward, left, turn, poseTracker.getPose().getRotation(),
        Constants.WHEELBASE_METERS, Constants.TRACKWIDTH_METERS);
    for (int i = 0; i < modules.length; i++) {
      modules[i].setDesiredState(requests[i].speed(), requests[i].angleRadians());
    }
  }

  // Speed here is a normalized motor request, NOT meters per second.
  public record WheelRequest(double speed, double angleRadians) {}

  /** Rotate the joystick translation into the robot frame; spin is unchanged. */
  public static WheelRequest[] calculateFieldRelativeWheelRequests(
      double forward, double left, double turn, Rotation2d heading,
      double wheelbase, double trackwidth) {
    double robotForward = forward * heading.getCos() + left * heading.getSin();
    double robotLeft = -forward * heading.getSin() + left * heading.getCos();
    return calculateWheelRequests(robotForward, robotLeft, turn, wheelbase, trackwidth);
  }

  /** Add translation and rotation vectors, then scale all wheels together. */
  public static WheelRequest[] calculateWheelRequests(
      double forward, double left, double turn, double wheelbase, double trackwidth) {
    double radius = Math.hypot(wheelbase / 2.0, trackwidth / 2.0);
    double x = (wheelbase / 2.0) / radius;
    double y = (trackwidth / 2.0) / radius;
    // Module positions: +x is forward, +y is left.
    double[] moduleX = {x, x, -x, -x};
    double[] moduleY = {y, -y, y, -y};
    WheelRequest[] requests = new WheelRequest[4];
    double largestSpeed = 1.0;
    for (int i = 0; i < requests.length; i++) {
      double wheelForward = forward - turn * moduleY[i];
      double wheelLeft = left + turn * moduleX[i];
      double speed = Math.hypot(wheelForward, wheelLeft);
      requests[i] = new WheelRequest(speed, Math.atan2(wheelLeft, wheelForward));
      largestSpeed = Math.max(largestSpeed, speed);
    }
    for (int i = 0; i < requests.length; i++) {
      requests[i] = new WheelRequest(requests[i].speed() / largestSpeed, requests[i].angleRadians());
    }
    return requests;
  }

  public void zeroSteering() {
    // Keep calibration impossible while enabled, even if called elsewhere later.
    if (!DriverStation.isDisabled() || !Constants.hardwareConfigured()) {
      return;
    }
    stop();
    for (SwerveModule module : modules) {
      module.zeroSteering();
    }
    sensorsReady = false;
    poseTracker.invalidate();
  }

  /** Called every robot loop, including while disabled and with centered sticks. */
  public void periodic() {
    sensorsReady = readSensors();
    if (sensorsReady) {
      poseTracker.update(rawHeading, positions);
    } else {
      if (poseTracker != null) {
        poseTracker.invalidate();
      }
      stop();
    }
    updateDashboard();
  }

  /** Triangle: make the current position (0,0) and current facing field-forward. */
  public void resetOdometry() {
    stop();
    sensorsReady = readSensors();
    if (sensorsReady) {
      poseTracker.reset(rawHeading, positions);
      updateDashboard();
    } else {
      if (poseTracker != null) {
        poseTracker.invalidate();
      }
      DriverStation.reportWarning(
          "Cannot reset pose: configure hardware, connect the gyro, and zero steering first.", false);
    }
  }

  private boolean readSensors() {
    if (!Constants.hardwareConfigured() || !gyro.isConnected()) {
      return false;
    }
    boolean modulesReady = true;
    for (SwerveModule module : modules) {
      modulesReady &= module.isReady();
    }
    double headingRadians = gyro.getHeadingRadians();
    if (!modulesReady || !Double.isFinite(headingRadians)) {
      return false;
    }
    rawHeading = new Rotation2d(headingRadians);
    for (int i = 0; i < modules.length; i++) {
      positions[i] = modules[i].getPosition();
      if (positions[i] == null) {
        return false;
      }
    }
    return true;
  }

  private boolean isReady() {
    return sensorsReady && poseTracker != null && poseTracker.isReady();
  }

  public void stop() {
    for (SwerveModule module : modules) {
      module.stop();
    }
  }

  private void updateDashboard() {
    SmartDashboard.putBoolean("Swerve/Hardware configured", Constants.hardwareConfigured());
    SmartDashboard.putBoolean("Swerve/Gyro connected", gyro.isConnected());
    SmartDashboard.putBoolean("Swerve/Ready", isReady());
    Pose2d pose = poseTracker == null ? new Pose2d() : poseTracker.getPose();
    SmartDashboard.putNumber("Swerve/X meters", pose.getX());
    SmartDashboard.putNumber("Swerve/Y meters", pose.getY());
    SmartDashboard.putNumber("Swerve/Heading degrees", pose.getRotation().getDegrees());
    field.setRobotPose(pose);
    for (SwerveModule module : modules) {
      module.updateDashboard();
    }
  }
}
