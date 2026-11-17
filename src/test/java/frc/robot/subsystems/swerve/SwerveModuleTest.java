package frc.robot.subsystems.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import frc.robot.Constants;
import org.junit.jupiter.api.Test;

class SwerveModuleTest {
  @Test
  void alignsToCalibratedForwardWithDriveStopped() {
    assertTrue(HAL.initialize(500, 0));
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    // Separate CAN IDs from the Robot startup test.
    var module = new SwerveModule("Test", 41, 42, false, false, 1.0, 1.0);
    var drive = new SimDeviceSim("SPARK MAX [41]").getDouble("Setpoint");
    var steer = new SimDeviceSim("SPARK MAX [42]").getDouble("Setpoint");
    var position = new SimDeviceSim("SPARK MAX [42] RELATIVE ENCODER").getDouble("Position");

    try {
      // An uncalibrated module must not steer or report alignment complete.
      assertFalse(module.pointForward());
      assertEquals(0.0, drive.get(), 1e-6);
      assertEquals(0.0, steer.get(), 1e-6);

      // A nonzero encoder baseline must remain the definition of forward.
      double baseline = 3.0;
      position.set(baseline);
      module.zeroSteering();
      assertTrue(module.isReady());
      DriverStationSim.setEnabled(true);
      DriverStationSim.notifyNewData();

      // Start with a drive command, then point forward from the backward direction.
      module.setDesiredState(0.5, 0.0);
      assertTrue(drive.get() > 0.0);
      position.set(baseline + Constants.STEER_GEAR_RATIO / 2.0);
      assertFalse(module.pointForward());
      assertEquals(0.0, drive.get(), 1e-6);
      assertEquals(Constants.MAX_STEER_OUTPUT, Math.abs(steer.get()), 1e-6);

      // Repeated calls recompute steering in both directions as the angle changes.
      for (double degrees : new double[] {90.0, -90.0}) {
        position.set(baseline + Constants.STEER_GEAR_RATIO * degrees / 360.0);
        assertFalse(module.pointForward());
        assertTrue(steer.get() * degrees < 0.0);
        assertEquals(0.0, drive.get(), 1e-6);
      }

      // Forward is periodic, and a small error is accepted without steering jitter.
      position.set(baseline + Constants.STEER_GEAR_RATIO * 361.0 / 360.0);
      assertTrue(module.pointForward());
      assertEquals(0.0, drive.get(), 1e-6);
      assertEquals(0.0, steer.get(), 1e-6);

      position.set(Double.NaN);
      assertFalse(module.pointForward());
      assertFalse(module.isReady());
      assertEquals(0.0, drive.get(), 1e-6);
      assertEquals(0.0, steer.get(), 1e-6);
    } finally {
      module.stop();
      DriverStationSim.setEnabled(false);
      DriverStationSim.notifyNewData();
    }
  }
}
