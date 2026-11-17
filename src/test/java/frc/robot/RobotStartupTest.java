package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.junit.jupiter.api.Test;

class RobotStartupTest {
  @Test
  void startsDisabledWithVendorLibrariesAndNoConnectedHardware() {
    assertTrue(HAL.initialize(500, 0));
    assertTrue(RobotBase.isSimulation());
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();

    // Constructing Robot loads both the REV and Redux native libraries.
    try (Robot robot = new Robot()) {
      robot.robotInit();
      robot.disabledInit();
      robot.robotPeriodic();
      robot.disabledPeriodic();

      // No CANcoder configuration/data must never masquerade as a forward module.
      for (String module : Constants.MODULE_NAMES) {
        assertFalse(SmartDashboard.getBoolean("Swerve/" + module + "/Zeroed", true));
        assertFalse(SmartDashboard.getBoolean("Swerve/" + module + "/Encoder healthy", true));
      }
      // A missing gyro must still prevent the drivetrain becoming ready.
      assertFalse(SmartDashboard.getBoolean("Swerve/Ready", true));

      DriverStationSim.setJoystickAxisCount(Constants.PS4_PORT, 6);
      DriverStationSim.setJoystickButtonCount(Constants.PS4_PORT, 14);
      DriverStationSim.setJoystickAxis(Constants.PS4_PORT, 1, -1.0);
      DriverStationSim.setEnabled(true);
      DriverStationSim.notifyNewData();
      robot.teleopInit();
      for (int i = 0; i < 3; i++) {
        robot.teleopPeriodic();
        robot.robotPeriodic();
      }
      assertFalse(SmartDashboard.getBoolean("Swerve/Ready", true));
      for (int id : Constants.DRIVE_CAN_IDS) {
        assertEquals(0.0, new SimDeviceSim("SPARK MAX [" + id + "]")
            .getDouble("Setpoint").get(), 1e-6);
      }
      for (int id : Constants.STEER_CAN_IDS) {
        assertEquals(0.0, new SimDeviceSim("SPARK MAX [" + id + "]")
            .getDouble("Setpoint").get(), 1e-6);
      }
    } finally {
      DriverStationSim.resetData();
      DriverStationSim.notifyNewData();
    }
  }
}
