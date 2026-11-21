package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import java.util.Arrays;

public class SwerveDrive {
  private final SwerveModule[] modules;
  private final RobotGyro gyro;
  private final SwervePoseTracker poseTracker;
  private final Field2d field = new Field2d();
  private Rotation2d rawHeading = new Rotation2d();
  private final SwerveModulePosition[] positions = new SwerveModulePosition[4];
  private boolean sensorsReady;
  private boolean poseResetPending;
  private boolean aligningForTeleop;

  public SwerveDrive() {
    this(new RobotGyro(), createModules());
  }

  SwerveDrive(RobotGyro gyro, SwerveModule[] modules) {
    this.gyro = gyro;
    this.modules = modules;
    poseTracker = Constants.hardwareConfigured()
        ? new SwervePoseTracker(Constants.WHEELBASE_METERS, Constants.TRACKWIDTH_METERS) : null;
    SmartDashboard.putData("Swerve/Field", field);
    if (!Constants.hardwareConfigured()) {
      DriverStation.reportWarning(
          "Swerve: set wheelbase, trackwidth, wheel diameter, and drive gear ratio in Constants.java.",
          false);
    }
  }

  private static SwerveModule[] createModules() {
    SwerveModule[] modules = new SwerveModule[4];
    for (int i = 0; i < modules.length; i++) {
      modules[i] = new SwerveModule(
          Constants.MODULE_NAMES[i], Constants.DRIVE_CAN_IDS[i], Constants.STEER_CAN_IDS[i],
          Constants.DRIVE_INVERTED[i], Constants.STEER_INVERTED[i], Constants.DRIVE_ENCODER_SIGN[i],
          validEncoderId(i), Constants.CANCODER_FORWARD_ROTATIONS[i], Constants.STEER_ENCODER_SIGN[i]);
    }
    return modules;
  }

  private static int validEncoderId(int index) {
    int id = Constants.CANCODER_CAN_IDS[index];
    for (int i = 0; i < Constants.CANCODER_CAN_IDS.length; i++) {
      if (i != index && Constants.CANCODER_CAN_IDS[i] == id) {
        return -1;
      }
    }
    return id;
  }

  public void drive(double forward, double left, double turn) {
    if (aligningForTeleop) {
      finishTeleopAlignment();
      return;
    }
    if (!isReady()) {
      stop();
      return;
    }

    if (forward == 0.0 && left == 0.0 && turn == 0.0) {
      pointWheelsForward();
      return;
    }
    WheelRequest[] requests = calculateFieldRelativeWheelRequests(
        forward, left, turn, poseTracker.getPose().getRotation(),
        Constants.WHEELBASE_METERS, Constants.TRACKWIDTH_METERS);
    for (int i = 0; i < modules.length; i++) {
      modules[i].setDesiredState(requests[i].speed(), requests[i].angleRadians());
    }
  }

  public void prepareForTeleop() {
    aligningForTeleop = true;
    resetOdometry();
    finishTeleopAlignment();
  }

  private void finishTeleopAlignment() {
    aligningForTeleop = !pointWheelsForward();
    updateDashboard();
  }

  public boolean pointWheelsForward() {
    if (!hasValidPose()) {
      stop();
      return false;
    }
    boolean aligned = true;
    for (SwerveModule module : modules) {
      aligned &= module.pointForward();
    }
    return aligned;
  }

  public record WheelRequest(double speed, double angleRadians) {}

  public static WheelRequest[] calculateFieldRelativeWheelRequests(
      double forward, double left, double turn, Rotation2d heading,
      double wheelbase, double trackwidth) {
    double robotForward = forward * heading.getCos() + left * heading.getSin();
    double robotLeft = -forward * heading.getSin() + left * heading.getCos();
    return calculateWheelRequests(robotForward, robotLeft, turn, wheelbase, trackwidth);
  }

  public static WheelRequest[] calculateWheelRequests(
      double forward, double left, double turn, double wheelbase, double trackwidth) {
    double radius = Math.hypot(wheelbase / 2.0, trackwidth / 2.0);
    double x = (wheelbase / 2.0) / radius;
    double y = (trackwidth / 2.0) / radius;
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

  public void resetForStartup() {
    stop();
    for (SwerveModule module : modules) {
      module.resetForStartup();
    }
    rawHeading = new Rotation2d();
    Arrays.fill(positions, null);
    sensorsReady = false;
    poseResetPending = true;
    aligningForTeleop = false;
    if (poseTracker != null) {
      poseTracker.clear();
    }
    updateDashboard();
  }

  public void periodic() {
    for (SwerveModule module : modules) {
      module.refreshSensors();
      module.initializeStartup();
    }
    sensorsReady = readSensors();
    if (sensorsReady) {
      if (poseResetPending) {
        poseTracker.reset(rawHeading, positions);
        poseResetPending = false;
      } else {
        poseTracker.update(rawHeading, positions);
      }
    } else {
      if (poseTracker != null) {
        poseTracker.invalidate();
      }
      stop();
    }
    updateDashboard();
  }

  public void resetOdometry() {
    stop();
    poseResetPending = true;
    periodic();
  }

  private boolean readSensors() {
    if (!Constants.hardwareConfigured()) {
      return false;
    }
    boolean modulesReady = true;
    for (SwerveModule module : modules) {
      modulesReady &= module.isReady();
    }
    double headingRadians = gyro.getHeadingRadians();
    if (!modulesReady || !gyro.isConnected() || !Double.isFinite(headingRadians)) {
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
    return hasValidPose() && !aligningForTeleop;
  }

  private boolean hasValidPose() {
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
    SmartDashboard.putString("Swerve/State", !sensorsReady ? "WAITING_FOR_SENSORS"
        : !hasValidPose() ? "RESET_REQUIRED" : aligningForTeleop ? "ALIGNING" : "READY");
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
