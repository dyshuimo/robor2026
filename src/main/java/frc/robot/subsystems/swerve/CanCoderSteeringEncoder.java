package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.MagnetHealthValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import frc.robot.Constants;

/** Reads absolute module angle without zeroing the CANcoder or overwriting Tuner offsets. */
final class CanCoderSteeringEncoder implements AbsoluteSteeringEncoder, AutoCloseable {
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
