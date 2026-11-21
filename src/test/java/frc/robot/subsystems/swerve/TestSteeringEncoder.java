package frc.robot.subsystems.swerve;

import frc.robot.subsystems.swerve.SwerveModule.AbsoluteSteeringEncoder;

class TestSteeringEncoder implements AbsoluteSteeringEncoder {
  double angleRadians;
  Status status = Status.OK;

  @Override
  public Reading read() {
    return new Reading(angleRadians / (2.0 * Math.PI), angleRadians, status);
  }
}
