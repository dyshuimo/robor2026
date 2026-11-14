package frc.robot.subsystems.swerve;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
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

/** One drive motor plus one steering motor with its built-in relative encoder. */
public class SwerveModule {
  private final String name;
  private final SparkMax driveMotor;
  private final SparkMax steerMotor;
  private final RelativeEncoder steerEncoder;
  private final RelativeEncoder driveEncoder;
  private final double encoderSign;
  private final double driveEncoderSign;
  private final boolean configurationOK;
  private double zeroMotorRotations;
  private boolean zeroed;
  private boolean zeroPending;

  public SwerveModule(String name, int driveId, int steerId,
      boolean driveInverted, boolean steerInverted, double encoderSign, double driveEncoderSign) {
    this.name = name;
    this.encoderSign = encoderSign;
    this.driveEncoderSign = driveEncoderSign;
    driveMotor = new SparkMax(driveId, MotorType.kBrushless);
    steerMotor = new SparkMax(steerId, MotorType.kBrushless);
    steerEncoder = steerMotor.getEncoder();
    driveEncoder = driveMotor.getEncoder();

    SparkMaxConfig driveConfig = new SparkMaxConfig();
    driveConfig.inverted(driveInverted).idleMode(IdleMode.kBrake)
        .smartCurrentLimit(Constants.DRIVE_CURRENT_LIMIT_AMPS).voltageCompensation(12.0);
    driveConfig.encoder.positionConversionFactor(1.0);
    driveConfig.signals.primaryEncoderPositionPeriodMs(20);
    SparkMaxConfig steerConfig = new SparkMaxConfig();
    // Coast lets you align the steering by hand while disabled.
    steerConfig.inverted(steerInverted).idleMode(IdleMode.kCoast)
        .smartCurrentLimit(Constants.STEER_CURRENT_LIMIT_AMPS).voltageCompensation(12.0);
    steerConfig.encoder.positionConversionFactor(1.0); // Raw motor rotations.
    steerConfig.signals.primaryEncoderPositionPeriodMs(20);

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

  /** Call only after physically aligning this module forward while disabled. */
  public void zeroSteering() {
    if (!DriverStation.isDisabled()) {
      return;
    }
    zeroed = false;
    zeroPending = false;
    stop();
    REVLibError clearResult = steerMotor.clearFaults();
    REVLibError driveClearResult = driveMotor.clearFaults();
    double position = steerEncoder.getPosition();
    if (configurationOK && clearResult == REVLibError.kOk && driveClearResult == REVLibError.kOk
        && steerMotor.getLastError() == REVLibError.kOk && Double.isFinite(position)) {
      zeroMotorRotations = position;
      // Wait for a CAN status update confirming the reset warning was cleared.
      zeroPending = true;
    } else {
      DriverStation.reportError("Swerve " + name + ": could not zero steering encoder.", false);
    }
  }

  public boolean isReady() {
    boolean hasReset = steerMotor.getStickyWarnings().hasReset
        || driveMotor.getStickyWarnings().hasReset;
    if (zeroPending && DriverStation.isDisabled() && !hasReset) {
      zeroed = true;
      zeroPending = false;
    }
    if (!DriverStation.isDisabled()) {
      zeroPending = false;
    }
    // A controller reboot loses its relative encoder reference. Re-align and zero.
    if (hasReset) {
      zeroed = false;
    }
    return configurationOK && zeroed;
  }

  public double getAngleRadians() {
    return encoderSign * (steerEncoder.getPosition() - zeroMotorRotations)
        * 2.0 * Math.PI / Constants.STEER_GEAR_RATIO;
  }

  /** Measured drive distance and steering angle for WPILib odometry. */
  public SwerveModulePosition getPosition() {
    double rotations = driveEncoder.getPosition();
    boolean driveOK = driveMotor.getLastError() == REVLibError.kOk;
    double angle = getAngleRadians();
    boolean steerOK = steerMotor.getLastError() == REVLibError.kOk;
    if (!driveOK || !steerOK || !Double.isFinite(rotations) || !Double.isFinite(angle)) {
      return null;
    }
    double meters = driveEncoderSign * rotations
        * Math.PI * Constants.WHEEL_DIAMETER_METERS / Constants.DRIVE_GEAR_RATIO;
    return new SwerveModulePosition(meters, new Rotation2d(angle));
  }

  public void setDesiredState(double speed, double targetAngle) {
    if (!isReady() || Math.abs(speed) < 0.001) {
      // Avoid steering jitter when the sticks are centered.
      stop();
      return;
    }
    double currentAngle = getAngleRadians();
    if (!Double.isFinite(currentAngle) || steerMotor.getLastError() != REVLibError.kOk) {
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
    SmartDashboard.putBoolean("Swerve/" + name + "/Zeroed", zeroed);
    if (Constants.hardwareConfigured()) {
      SmartDashboard.putNumber("Swerve/" + name + "/Angle degrees", Math.toDegrees(getAngleRadians()));
      SmartDashboard.putNumber("Swerve/" + name + "/Drive meters",
          driveEncoderSign * driveEncoder.getPosition()
              * Math.PI * Constants.WHEEL_DIAMETER_METERS / Constants.DRIVE_GEAR_RATIO);
    }
  }
}
