package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.MagnetHealthValue;
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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

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

  public void resetForStartup() {
    stop();
    steering = new AbsoluteSteeringEncoder.Reading(
        Double.NaN, Double.NaN, AbsoluteSteeringEncoder.Status.CAN_ERROR);
    zeroDriveRotations = 0.0;
    zeroed = false;
    zeroPending = false;
    startupResetPending = true;
  }

  public void refreshSensors() {
    steering = steerEncoder.read();
  }

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
    if (hasReset) {
      zeroed = false;
    }
    return configurationOK && zeroed && steering.valid();
  }

  public double getAngleRadians() {
    return steering.angleRadians();
  }

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
    if (Math.abs(error) > Math.PI / 2.0) {
      speed = -speed;
      error = MathUtil.angleModulus(error + Math.PI);
    }
    double steerOutput = MathUtil.clamp(
        Constants.STEER_KP * error, -Constants.MAX_STEER_OUTPUT, Constants.MAX_STEER_OUTPUT);
    steerMotor.set(steerOutput);
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

  @FunctionalInterface
  interface AbsoluteSteeringEncoder {
    enum Status { OK, UNCONFIGURED_ID, UNCONFIGURED_FORWARD, CAN_ERROR, STALE, MAGNET_FAULT,
      INVALID_READING }

    record Reading(double absoluteRotations, double angleRadians, Status status) {
      boolean valid() {
        return status == Status.OK && Double.isFinite(absoluteRotations)
            && Double.isFinite(angleRadians);
      }
    }

    Reading read();
  }

  static final class CanCoderSteeringEncoder implements AbsoluteSteeringEncoder, AutoCloseable {
    private final CANcoder encoder;
    private final StatusSignal<Angle> position;
    private final StatusSignal<MagnetHealthValue> magnet;
    private final double forwardRotations;
    private final double sign;

    CanCoderSteeringEncoder(int canId, double forwardRotations, double sign) {
      this(canId >= 0 && canId <= 62 ? new CANcoder(canId, new CANBus("rio")) : null,
          forwardRotations, sign);
    }

    CanCoderSteeringEncoder(CANcoder encoder, double forwardRotations, double sign) {
      this.encoder = encoder;
      this.forwardRotations = forwardRotations;
      this.sign = sign;
      if (encoder == null) {
        position = null;
        magnet = null;
        return;
      }
      position = encoder.getAbsolutePosition(false);
      magnet = encoder.getMagnetHealth(false);
      BaseStatusSignal.setUpdateFrequencyForAll(50.0, position, magnet);
    }

    @Override
    public Reading read() {
      if (encoder == null) {
        return new Reading(Double.NaN, Double.NaN, Status.UNCONFIGURED_ID);
      }
      BaseStatusSignal.refreshAll(position, magnet);
      return evaluate(position.getValueAsDouble(), forwardRotations, sign,
          BaseStatusSignal.isAllGood(position, magnet),
          position.getTimestamp().isValid() && magnet.getTimestamp().isValid(),
          Math.max(position.getTimestamp().getLatency(), magnet.getTimestamp().getLatency()),
          magnet.getValue());
    }

    static Reading evaluate(double rotations, double forward, double sign,
        boolean communicationOK, boolean timestampValid, double age, MagnetHealthValue health) {
      Status status;
      if (!communicationOK) {
        status = Status.CAN_ERROR;
      } else if (!timestampValid || !Double.isFinite(age) || age < 0.0
          || age > Constants.CANCODER_MAX_AGE_SECONDS) {
        status = Status.STALE;
      } else if (health != MagnetHealthValue.Magnet_Green
          && health != MagnetHealthValue.Magnet_Orange) {
        status = Status.MAGNET_FAULT;
      } else if (!Double.isFinite(forward)) {
        status = Status.UNCONFIGURED_FORWARD;
      } else if (!Double.isFinite(rotations) || (sign != 1.0 && sign != -1.0)) {
        status = Status.INVALID_READING;
      } else {
        return new Reading(rotations, MathUtil.angleModulus(sign * (rotations - forward)
            * 2.0 * Math.PI), Status.OK);
      }
      return new Reading(rotations, Double.NaN, status);
    }

    @Override
    public void close() {
      if (encoder != null) {
        encoder.close();
      }
    }
  }
}
