package frc.robot.subsystems.swerve;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/** Two SPARK MAX motors, a CANcoder for azimuth, and the drive motor's travel encoder. */
public class SwerveModule {
  private final String name;
  private final SparkMax driveMotor;
  private final SparkMax steerMotor;
  private final AbsoluteSteeringEncoder steerEncoder;
  private AbsoluteSteeringEncoder.Reading steering = new AbsoluteSteeringEncoder.Reading(
      Double.NaN, Double.NaN, AbsoluteSteeringEncoder.Status.CAN_ERROR);
  private final RelativeEncoder driveEncoder;
  private final double driveEncoderSign;
  private final boolean configurationOK;
  private double zeroDriveRotations;
  private boolean zeroed;
  private boolean zeroPending;
  private boolean startupResetPending;

  public SwerveModule(String name, int driveId, int steerId,
      boolean driveInverted, boolean steerInverted, double driveEncoderSign,
      int canCoderId, double forwardRotations, double encoderSign) {
    this(name, driveId, steerId, driveInverted, steerInverted, driveEncoderSign,
        new CanCoderSteeringEncoder(canCoderId, forwardRotations, encoderSign));
  }

  SwerveModule(String name, int driveId, int steerId,
      boolean driveInverted, boolean steerInverted, double driveEncoderSign,
      AbsoluteSteeringEncoder steerEncoder) {
    this.name = name;
    this.steerEncoder = steerEncoder;
    this.driveEncoderSign = driveEncoderSign;
    driveMotor = new SparkMax(driveId, MotorType.kBrushless);
    steerMotor = new SparkMax(steerId, MotorType.kBrushless);
    driveEncoder = driveMotor.getEncoder();

    SparkMaxConfig driveConfig = new SparkMaxConfig();
    driveConfig.inverted(driveInverted).idleMode(IdleMode.kBrake)
        .smartCurrentLimit(Constants.DRIVE_CURRENT_LIMIT_AMPS).voltageCompensation(12.0);
    driveConfig.encoder.positionConversionFactor(1.0);
    driveConfig.signals.primaryEncoderPositionPeriodMs(20);
    SparkMaxConfig steerConfig = new SparkMaxConfig();
    // Steering coasts when no output is requested.
    steerConfig.inverted(steerInverted).idleMode(IdleMode.kCoast)
        .smartCurrentLimit(Constants.STEER_CURRENT_LIMIT_AMPS).voltageCompensation(12.0);

    REVLibError driveResult = driveMotor.configure(
        driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    REVLibError steerResult = steerMotor.configure(
        steerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    configurationOK = driveResult == REVLibError.kOk && steerResult == REVLibError.kOk;
    if (!configurationOK) {
      DriverStation.reportError("Swerve " + name + " configuration failed: drive="
          + driveResult + ", steer=" + steerResult, false);
    }
  }

  /** Reset drive travel only; the calibrated CANcoder forward reference survives startup. */
  public void resetForStartup() {
    stop();
    steering = new AbsoluteSteeringEncoder.Reading(
        Double.NaN, Double.NaN, AbsoluteSteeringEncoder.Status.CAN_ERROR);
    zeroDriveRotations = 0.0;
    zeroed = false;
    zeroPending = false;
    startupResetPending = true;
  }

  /** Capture a coherent steering sample for this control loop. */
  public void refreshSensors() {
    steering = steerEncoder.read();
  }

  /** Retry initial sensor reads automatically, including if enable precedes CAN startup. */
  public void initializeStartup() {
    if (!startupResetPending || zeroPending || !configurationOK || !steering.valid()) {
      return;
    }
    stop();
    REVLibError clearResult = steerMotor.clearFaults();
    REVLibError driveClearResult = driveMotor.clearFaults();
    double drivePosition = driveEncoder.getPosition();
    boolean driveOK = driveMotor.getLastError() == REVLibError.kOk;
    if (clearResult == REVLibError.kOk && driveClearResult == REVLibError.kOk
        && driveOK && Double.isFinite(drivePosition)) {
      zeroDriveRotations = drivePosition;
      // Wait for a CAN status update confirming the reset warning was cleared.
      zeroPending = true;
    }
  }

  public boolean isReady() {
    var steerWarnings = steerMotor.getStickyWarnings();
    boolean steerOK = steerMotor.getLastError() == REVLibError.kOk;
    var driveWarnings = driveMotor.getStickyWarnings();
    boolean driveOK = driveMotor.getLastError() == REVLibError.kOk;
    if (!steerOK || !driveOK) {
      return false;
    }
    boolean hasReset = steerWarnings.hasReset || driveWarnings.hasReset;
    if (zeroPending && !hasReset) {
      zeroed = true;
      zeroPending = false;
      startupResetPending = false;
    }
    // A later controller reboot loses the reference; restart initialization before driving.
    if (hasReset) {
      zeroed = false;
    }
    return configurationOK && zeroed && steering.valid();
  }

  public double getAngleRadians() {
    return steering.angleRadians();
  }

  /** Measured drive distance and steering angle for WPILib odometry. */
  public SwerveModulePosition getPosition() {
    double rotations = driveEncoder.getPosition();
    boolean driveOK = driveMotor.getLastError() == REVLibError.kOk;
    double angle = getAngleRadians();
    if (!driveOK || !steering.valid() || !Double.isFinite(rotations)) {
      return null;
    }
    double meters = driveEncoderSign * (rotations - zeroDriveRotations)
        * Math.PI * Constants.WHEEL_DIAMETER_METERS / Constants.DRIVE_GEAR_RATIO;
    return new SwerveModulePosition(meters, new Rotation2d(angle));
  }

  /** Steer to the calibrated forward direction; return true once within tolerance. */
  public boolean pointForward() {
    driveMotor.stopMotor();
    if (!isReady()) {
      stop();
      return false;
    }
    double currentAngle = getAngleRadians();
    if (!Double.isFinite(currentAngle)) {
      zeroed = false;
      stop();
      return false;
    }
    // Do not optimize by 180 degrees: use the calibrated physical forward reference.
    double error = MathUtil.angleModulus(-currentAngle);
    if (Math.abs(error) <= Constants.STEER_ALIGNMENT_TOLERANCE_RADIANS) {
      steerMotor.stopMotor();
      return true;
    }
    steerMotor.set(MathUtil.clamp(
        Constants.STEER_KP * error, -Constants.MAX_STEER_OUTPUT, Constants.MAX_STEER_OUTPUT));
    return false;
  }

  public void setDesiredState(double speed, double targetAngle) {
    if (!isReady() || Math.abs(speed) < 0.001) {
      // Avoid steering jitter when the sticks are centered.
      stop();
      return;
    }
    double currentAngle = getAngleRadians();
    if (!Double.isFinite(currentAngle)) {
      zeroed = false;
      stop();
      return;
    }
    double error = MathUtil.angleModulus(targetAngle - currentAngle);
    // Turn at most 90 degrees; reverse the drive motor for the other half-circle.
    if (Math.abs(error) > Math.PI / 2.0) {
      speed = -speed;
      error = MathUtil.angleModulus(error + Math.PI);
    }
    double steerOutput = MathUtil.clamp(
        Constants.STEER_KP * error, -Constants.MAX_STEER_OUTPUT, Constants.MAX_STEER_OUTPUT);
    steerMotor.set(steerOutput);
    // Reduce drive while the wheel is still turning toward its requested angle.
    driveMotor.set(speed * Constants.MAX_DRIVE_OUTPUT * Math.max(0.0, Math.cos(error)));
  }

  public void stop() {
    driveMotor.stopMotor();
    steerMotor.stopMotor();
  }

  public void updateDashboard() {
    SmartDashboard.putBoolean("Swerve/" + name + "/Zeroed", zeroed && steering.valid());
    SmartDashboard.putBoolean("Swerve/" + name + "/Encoder healthy", steering.valid());
    SmartDashboard.putString("Swerve/" + name + "/Encoder status", steering.status().name());
    SmartDashboard.putNumber("Swerve/" + name + "/Absolute rotations", steering.absoluteRotations());
    SmartDashboard.putBoolean("Swerve/" + name + "/Aligned forward",
        isReady() && Math.abs(getAngleRadians()) <= Constants.STEER_ALIGNMENT_TOLERANCE_RADIANS);
    if (Constants.hardwareConfigured()) {
      SmartDashboard.putNumber("Swerve/" + name + "/Angle degrees", Math.toDegrees(getAngleRadians()));
      SmartDashboard.putNumber("Swerve/" + name + "/Drive meters",
          driveEncoderSign * (driveEncoder.getPosition() - zeroDriveRotations)
              * Math.PI * Constants.WHEEL_DIAMETER_METERS / Constants.DRIVE_GEAR_RATIO);
    }
  }
}
