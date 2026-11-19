package frc.robot.subsystems.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveModule.AbsoluteSteeringEncoder;
import org.junit.jupiter.api.Test;

class SwerveStartupTest {
  @Test
  void automaticallyInitializesAndCompletesTeleopResetWhenGyroArrives() {
    assertTrue(HAL.initialize(500, 0));
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    var gyro = new StartupGyro();
    var modules = new SwerveModule[4];
    var steering = new TestSteeringEncoder[4];
    for (int i = 0; i < modules.length; i++) {
      steering[i] = new TestSteeringEncoder();
      steering[i].angleRadians = Math.PI / 2.0;
      modules[i] = new SwerveModule("Startup" + i, 51 + i * 2, 52 + i * 2,
          false, false, 1.0, steering[i]);
      encoder(51 + i * 2).getDouble("Position").set(20.0);
    }
    var swerve = new SwerveDrive(gyro, modules);

    try {
      swerve.resetForStartup();
      // Enable before the first initialization loop: no disabled calibration is required.
      DriverStationSim.setEnabled(true);
      DriverStationSim.notifyNewData();
      swerve.prepareForTeleop();
      for (SwerveModule module : modules) {
        assertTrue(module.isReady());
        assertEquals(Math.PI / 2.0, module.getAngleRadians(), 1e-9);
        assertEquals(0.0, module.getPosition().distanceMeters, 1e-9);
      }
      assertFalse(ready());
      swerve.drive(1, 0, 0);
      assertDriveOutput(0.0);

      // A connected but calibrating gyro is not a usable heading yet.
      gyro.connected = true;
      swerve.periodic();
      assertFalse(ready());
      gyro.heading = Math.toRadians(70);
      swerve.periodic();
      assertFalse(ready());
      swerve.drive(1, 0, 0);
      assertFalse(ready());
      assertEquals("ALIGNING", SmartDashboard.getString("Swerve/State", ""));
      assertDriveOutput(0.0);
      for (var sample : steering) {
        sample.angleRadians = 0.0;
      }
      swerve.periodic();
      swerve.drive(1, 0, 0);
      assertTrue(ready());
      assertOrigin();
      assertDriveOutput(0.0);
      // Measured alignment is complete, so the next command can drive without a button press.
      swerve.drive(1, 0, 0);
      assertDriveOutput(Constants.MAX_DRIVE_OUTPUT);

      double rotationsPerMeter = Constants.DRIVE_GEAR_RATIO
          / (Math.PI * Constants.WHEEL_DIAMETER_METERS);
      for (int i = 0; i < modules.length; i++) {
        encoder(51 + i * 2).getDouble("Position").set(20.0 + rotationsPerMeter);
      }
      swerve.periodic();
      assertEquals(1.0, SmartDashboard.getNumber("Swerve/X meters", -1), 1e-6);

      // A later Teleop reset also completes automatically if the gyro is briefly unavailable.
      gyro.connected = false;
      swerve.resetOdometry();
      assertFalse(ready());
      assertDriveOutput(0.0);
      gyro.connected = true;
      swerve.periodic();
      assertTrue(ready());
      assertOrigin();

      // Resetting the pose on re-enable must preserve the original module forward references.
      for (int i = 0; i < modules.length; i++) {
        steering[i].angleRadians = Math.PI / 2.0;
      }
      swerve.resetOdometry();
      for (SwerveModule module : modules) {
        assertEquals(Math.PI / 2, module.getAngleRadians(), 1e-6);
      }

      // Every Teleop entry starts steering immediately, using the original forward references.
      swerve.prepareForTeleop();
      assertFalse(ready());
      assertDriveOutput(0.0);
      for (int i = 0; i < modules.length; i++) {
        assertTrue(new SimDeviceSim("SPARK MAX [" + (52 + i * 2) + "]")
            .getDouble("Setpoint").get() < 0.0);
      }
      // Even full joystick input must wait for the last module to finish alignment.
      for (int i = 0; i < modules.length - 1; i++) {
        steering[i].angleRadians = 0.0;
      }
      swerve.periodic();
      swerve.drive(1, 0, 0);
      assertFalse(ready());
      assertDriveOutput(0.0);
      steering[3].angleRadians = 0.0;
      swerve.periodic();
      swerve.drive(1, 0, 0);
      assertTrue(ready());
      assertDriveOutput(0.0);
      swerve.drive(1, 0, 0);
      assertDriveOutput(Constants.MAX_DRIVE_OUTPUT);

      // Losing one CANcoder stops the entire drivetrain and removes readiness.
      steering[3].status = AbsoluteSteeringEncoder.Status.STALE;
      swerve.periodic();
      swerve.drive(1, 0, 0);
      assertFalse(ready());
      assertDriveOutput(0.0);
      assertEquals("WAITING_FOR_SENSORS", SmartDashboard.getString("Swerve/State", ""));
      steering[3].status = AbsoluteSteeringEncoder.Status.OK;
      swerve.periodic();
      assertFalse(ready());
      assertEquals("RESET_REQUIRED", SmartDashboard.getString("Swerve/State", ""));

      // A new power-on reset clears travel and pose, but does not redefine physical angles.
      for (var sample : steering) {
        sample.angleRadians = -Math.PI / 2.0;
      }
      swerve.resetForStartup();
      swerve.periodic();
      assertTrue(ready());
      assertOrigin();
      for (SwerveModule module : modules) {
        assertEquals(-Math.PI / 2.0, module.getAngleRadians(), 1e-9);
        assertEquals(0.0, module.getPosition().distanceMeters, 1e-9);
      }
    } finally {
      swerve.stop();
      DriverStationSim.resetData();
      DriverStationSim.notifyNewData();
    }
  }

  private static SimDeviceSim encoder(int id) {
    return new SimDeviceSim("SPARK MAX [" + id + "] RELATIVE ENCODER");
  }

  private static boolean ready() {
    return SmartDashboard.getBoolean("Swerve/Ready", false);
  }

  private static void assertOrigin() {
    assertEquals(0.0, SmartDashboard.getNumber("Swerve/X meters", -1), 1e-9);
    assertEquals(0.0, SmartDashboard.getNumber("Swerve/Y meters", -1), 1e-9);
    assertEquals(0.0, SmartDashboard.getNumber("Swerve/Heading degrees", -1), 1e-9);
  }

  private static void assertDriveOutput(double expected) {
    for (int i = 0; i < 4; i++) {
      assertEquals(expected, new SimDeviceSim("SPARK MAX [" + (51 + i * 2) + "]")
          .getDouble("Setpoint").get(), 1e-6);
    }
  }

  private static class StartupGyro extends RobotGyro {
    boolean connected;
    double heading = Double.NaN;

    @Override
    public boolean isConnected() {
      return connected;
    }

    @Override
    public double getHeadingRadians() {
      return heading;
    }
  }
}
