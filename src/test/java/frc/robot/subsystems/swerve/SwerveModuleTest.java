package frc.robot.subsystems.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.MagnetHealthValue;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveModule.AbsoluteSteeringEncoder;
import frc.robot.subsystems.swerve.SwerveModule.AbsoluteSteeringEncoder.Status;
import frc.robot.subsystems.swerve.SwerveModule.CanCoderSteeringEncoder;
import org.junit.jupiter.api.Test;

class SwerveModuleTest {
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

  @Test
  void retriesStartupWhenEncoderDataBecomesValidAfterEnable() {
    assertTrue(HAL.initialize(500, 0));
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    var encoder = new TestSteeringEncoder();
    var module = new SwerveModule("DelayedStartup", 43, 44, false, false, 1.0, encoder);

    try {
      encoder.status = AbsoluteSteeringEncoder.Status.CAN_ERROR;
      module.resetForStartup();
      module.refreshSensors();
      module.initializeStartup();
      assertFalse(module.isReady());
      module.setDesiredState(1, 0);
      assertEquals(0.0, new SimDeviceSim("SPARK MAX [43]").getDouble("Setpoint").get());

      DriverStationSim.setEnabled(true);
      DriverStationSim.notifyNewData();
      encoder.status = AbsoluteSteeringEncoder.Status.OK;
      module.refreshSensors();
      module.initializeStartup();
      assertTrue(module.isReady());
      assertEquals(0.0, module.getAngleRadians(), 1e-9);
      module.setDesiredState(1, 0);
      assertEquals(Constants.MAX_DRIVE_OUTPUT,
          new SimDeviceSim("SPARK MAX [43]").getDouble("Setpoint").get(), 1e-6);

      encoder.status = AbsoluteSteeringEncoder.Status.STALE;
      module.refreshSensors();
      module.setDesiredState(1, 0);
      assertFalse(module.isReady());
      assertEquals(0.0, new SimDeviceSim("SPARK MAX [43]").getDouble("Setpoint").get());
    } finally {
      module.stop();
      DriverStationSim.resetData();
      DriverStationSim.notifyNewData();
    }
  }

  @Test
  void alignsToCalibratedForwardWithDriveStopped() {
    assertTrue(HAL.initialize(500, 0));
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    // Separate CAN IDs from the Robot startup test.
    var encoder = new TestSteeringEncoder();
    var module = new SwerveModule("Test", 41, 42, false, false, 1.0, encoder);
    var drive = new SimDeviceSim("SPARK MAX [41]").getDouble("Setpoint");
    var steer = new SimDeviceSim("SPARK MAX [42]").getDouble("Setpoint");

    try {
      // An uncalibrated module must not steer or report alignment complete.
      assertFalse(module.pointForward());
      assertEquals(0.0, drive.get(), 1e-6);
      assertEquals(0.0, steer.get(), 1e-6);

      // Startup at a nonzero absolute angle must not redefine physical forward.
      encoder.angleRadians = Math.PI;
      module.resetForStartup();
      module.refreshSensors();
      module.initializeStartup();
      assertTrue(module.isReady());
      DriverStationSim.setEnabled(true);
      DriverStationSim.notifyNewData();

      // Start with a drive command, then point forward from the backward direction.
      module.setDesiredState(0.5, Math.PI);
      assertTrue(drive.get() > 0.0);
      assertFalse(module.pointForward());
      assertEquals(0.0, drive.get(), 1e-6);
      assertEquals(Constants.MAX_STEER_OUTPUT, Math.abs(steer.get()), 1e-6);

      // Repeated calls recompute steering in both directions as the angle changes.
      for (double degrees : new double[] {90.0, -90.0}) {
        encoder.angleRadians = Math.toRadians(degrees);
        module.refreshSensors();
        assertFalse(module.pointForward());
        assertTrue(steer.get() * degrees < 0.0);
        assertEquals(0.0, drive.get(), 1e-6);
      }

      // Both alignment and the dashboard accept the configured inclusive tolerance.
      double tolerance = Constants.STEER_ALIGNMENT_TOLERANCE_RADIANS;
      for (double angle : new double[] {-tolerance, -tolerance / 2, 0, tolerance / 2, tolerance}) {
        encoder.angleRadians = angle;
        module.refreshSensors();
        assertTrue(module.pointForward());
        module.updateDashboard();
        assertTrue(SmartDashboard.getBoolean("Swerve/Test/Aligned forward", false));
        assertEquals(0.0, drive.get(), 1e-6);
        assertEquals(0.0, steer.get(), 1e-6);
      }

      // Just outside either boundary, keep steering with the drive motor stopped.
      for (double angle : new double[] {-tolerance - 0.0001, tolerance + 0.0001}) {
        encoder.angleRadians = angle;
        module.refreshSensors();
        assertFalse(module.pointForward());
        module.updateDashboard();
        assertFalse(SmartDashboard.getBoolean("Swerve/Test/Aligned forward", true));
        assertTrue(steer.get() * angle < 0.0);
        assertEquals(0.0, drive.get(), 1e-6);
      }

      encoder.angleRadians = Double.NaN;
      module.refreshSensors();
      assertFalse(module.pointForward());
      assertFalse(module.isReady());
      module.updateDashboard();
      assertFalse(SmartDashboard.getBoolean("Swerve/Test/Aligned forward", true));
      assertEquals(0.0, drive.get(), 1e-6);
      assertEquals(0.0, steer.get(), 1e-6);
    } finally {
      module.stop();
      DriverStationSim.setEnabled(false);
      DriverStationSim.notifyNewData();
    }
  }
}
