package frc.robot.subsystems.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.MagnetHealthValue;
import edu.wpi.first.hal.HAL;
import frc.robot.subsystems.swerve.AbsoluteSteeringEncoder.Status;
import org.junit.jupiter.api.Test;

class CanCoderSteeringEncoderTest {
  @Test
  void appliesForwardOffsetDirectionAndAngleWrapping() {
    var reading = evaluate(0.375, 0.125, 1, true, true, 0.01, MagnetHealthValue.Magnet_Green);
    assertTrue(reading.valid());
    assertEquals(Math.PI / 2, reading.angleRadians(), 1e-9);
    assertEquals(-Math.PI / 2,
        evaluate(0.375, 0.125, -1, true, true, 0.01, MagnetHealthValue.Magnet_Green)
            .angleRadians(), 1e-9);
    // Across the absolute wrap, a small positive turn remains a small positive angle.
    assertEquals(Math.toRadians(7.2),
        evaluate(0.01, 0.99, 1, true, true, 0.01, MagnetHealthValue.Magnet_Green)
            .angleRadians(), 1e-9);
  }

  @Test
  void rejectsCommunicationStalenessBadMagnetsAndMissingCalibration() {
    assertEquals(Status.CAN_ERROR,
        evaluate(0, 0, 1, false, true, 0, MagnetHealthValue.Magnet_Green).status());
    assertEquals(Status.STALE,
        evaluate(0, 0, 1, true, false, 0, MagnetHealthValue.Magnet_Green).status());
    assertEquals(Status.STALE,
        evaluate(0, 0, 1, true, true, 0.11, MagnetHealthValue.Magnet_Green).status());
    for (var health : new MagnetHealthValue[] {
        MagnetHealthValue.Magnet_Red, MagnetHealthValue.Magnet_Invalid}) {
      var reading = evaluate(0, 0, 1, true, true, 0, health);
      assertEquals(Status.MAGNET_FAULT, reading.status());
      assertFalse(reading.valid());
      assertTrue(Double.isNaN(reading.angleRadians()));
    }
    assertTrue(evaluate(0, 0, 1, true, true, 0, MagnetHealthValue.Magnet_Orange).valid());
    assertEquals(Status.UNCONFIGURED_FORWARD,
        evaluate(0, Double.NaN, 1, true, true, 0, MagnetHealthValue.Magnet_Green).status());
    assertEquals(Status.INVALID_READING,
        evaluate(Double.NaN, 0, 1, true, true, 0, MagnetHealthValue.Magnet_Green).status());
  }

  @Test
  void unknownIdNeverReportsAZeroAngle() {
    try (var encoder = new CanCoderSteeringEncoder(-1, 0, 1)) {
      var reading = encoder.read();
      assertEquals(Status.UNCONFIGURED_ID, reading.status());
      assertFalse(reading.valid());
      assertTrue(Double.isNaN(reading.angleRadians()));
    }
  }

  @Test
  void readsPhoenixSimulationAndDetectsMagnetFault() throws InterruptedException {
    assertTrue(HAL.initialize(500, 0));
    var device = new CANcoder(61, new CANBus("rio"));
    try (var encoder = new CanCoderSteeringEncoder(device, 0.125, 1)) {
      var sim = device.getSimState();
      assertTrue(sim.setSupplyVoltage(12).isOK());
      assertTrue(sim.setMagnetHealth(MagnetHealthValue.Magnet_Green).isOK());
      assertTrue(sim.setRawPosition(0.375).isOK());
      AbsoluteSteeringEncoder.Reading reading = encoder.read();
      // Phoenix publishes asynchronously; the first healthy frame may predate setRawPosition.
      for (int i = 0; i < 50 && (!reading.valid()
          || Math.abs(reading.absoluteRotations() - 0.375) > 1e-4); i++) {
        Thread.sleep(20);
        reading = encoder.read();
      }
      assertTrue(reading.valid(), reading.status().name());
      assertEquals(Math.PI / 2, reading.angleRadians(), 1e-3);
      assertTrue(sim.setMagnetHealth(MagnetHealthValue.Magnet_Red).isOK());
      for (int i = 0; i < 50 && reading.status() != Status.MAGNET_FAULT; i++) {
        Thread.sleep(20);
        reading = encoder.read();
      }
      assertEquals(Status.MAGNET_FAULT, reading.status());
      assertFalse(reading.valid());
    }
  }

  private static AbsoluteSteeringEncoder.Reading evaluate(double angle, double offset, double sign,
      boolean ok, boolean timestampValid, double age, MagnetHealthValue health) {
    return CanCoderSteeringEncoder.evaluate(angle, offset, sign, ok, timestampValid, age, health);
  }
}
