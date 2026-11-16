package frc.robot;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
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
      robot.disabledInit();
      robot.robotPeriodic();
      robot.disabledPeriodic();

      // Missing gyro/module references must prevent the drivetrain becoming ready.
      assertFalse(SmartDashboard.getBoolean("Swerve/Ready", true));
    }
  }
}
