package frc.robot.subsystems.swerve;

/** One measured module azimuth, independent of the drive motor's travel encoder. */
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
