package frc.robot.subsystems.swerve;

/** Deterministic encoder samples for module and drivetrain fault tests. */
class TestSteeringEncoder implements AbsoluteSteeringEncoder {
  double angleRadians;
  Status status = Status.OK;

  @Override
  public Reading read() {
    return new Reading(angleRadians / (2.0 * Math.PI), angleRadians, status);
  }
}
