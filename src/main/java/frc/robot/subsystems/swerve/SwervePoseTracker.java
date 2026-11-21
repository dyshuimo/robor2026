package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class SwervePoseTracker {
  private final SwerveDriveKinematics kinematics;
  private SwerveDriveOdometry odometry;
  private boolean needsReset;

  public SwervePoseTracker(double wheelbase, double trackwidth) {
    kinematics = new SwerveDriveKinematics(
        new Translation2d(wheelbase / 2.0, trackwidth / 2.0),
        new Translation2d(wheelbase / 2.0, -trackwidth / 2.0),
        new Translation2d(-wheelbase / 2.0, trackwidth / 2.0),
        new Translation2d(-wheelbase / 2.0, -trackwidth / 2.0));
  }

  public void update(Rotation2d rawHeading, SwerveModulePosition[] positions) {
    if (odometry == null) {
      reset(rawHeading, positions);
    } else if (!needsReset) {
      odometry.update(rawHeading, positions);
    }
  }

  public void reset(Rotation2d rawHeading, SwerveModulePosition[] positions) {
    if (odometry == null) {
      odometry = new SwerveDriveOdometry(kinematics, rawHeading, positions, new Pose2d());
    } else {
      odometry.resetPosition(rawHeading, positions, new Pose2d());
    }
    needsReset = false;
  }

  public void invalidate() {
    needsReset = true;
  }

  public void clear() {
    odometry = null;
    needsReset = true;
  }

  public boolean isReady() {
    return odometry != null && !needsReset;
  }

  public Pose2d getPose() {
    return odometry == null ? new Pose2d() : odometry.getPoseMeters();
  }
}
