package frc.robot.subsystems.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import org.junit.jupiter.api.Test;

class SwervePoseTrackerTest {
  private static final double EPSILON = 1e-9;

  @Test
  void initialSensorValuesAreBaselinesRatherThanDistanceAlreadyDriven() {
    var tracker = new SwervePoseTracker(0.8, 0.6);
    assertFalse(tracker.isReady());
    tracker.update(Rotation2d.fromDegrees(73), positions(10, 0));
    assertTrue(tracker.isReady());
    assertOrigin(tracker.getPose());
    tracker.update(Rotation2d.fromDegrees(73), positions(11, 0));
    assertEquals(1, tracker.getPose().getX(), EPSILON);
    assertEquals(0, tracker.getPose().getY(), EPSILON);
  }

  @Test
  void sidewaysWheelTravelUpdatesFieldY() {
    var tracker = new SwervePoseTracker(0.8, 0.6);
    tracker.update(new Rotation2d(), positions(0, 90));
    tracker.update(new Rotation2d(), positions(1.5, 90));
    assertEquals(0, tracker.getPose().getX(), EPSILON);
    assertEquals(1.5, tracker.getPose().getY(), EPSILON);
  }

  @Test
  void triangleClearsTranslationAndHeadingWithoutAJumpOnTheNextUpdate() {
    var tracker = new SwervePoseTracker(0.8, 0.6);
    tracker.update(Rotation2d.fromDegrees(35), positions(5, 0));
    tracker.update(Rotation2d.fromDegrees(35), positions(7, 0));
    tracker.update(Rotation2d.fromDegrees(95), positions(7, 0));
    assertEquals(2, tracker.getPose().getX(), EPSILON);
    assertEquals(60, tracker.getPose().getRotation().getDegrees(), EPSILON);

    tracker.reset(Rotation2d.fromDegrees(95), positions(7, 0));
    assertOrigin(tracker.getPose());
    tracker.update(Rotation2d.fromDegrees(95), positions(7, 0));
    assertOrigin(tracker.getPose());

    tracker.update(Rotation2d.fromDegrees(95), positions(8, 0));
    assertEquals(1, tracker.getPose().getX(), EPSILON);
    assertEquals(0, tracker.getPose().getY(), EPSILON);
    tracker.update(Rotation2d.fromDegrees(125), positions(8, 0));
    assertEquals(30, tracker.getPose().getRotation().getDegrees(), EPSILON);
  }

  @Test
  void fieldControlsUseTheSameHeadingReferenceAsTriangleReset() {
    var tracker = new SwervePoseTracker(0.8, 0.6);
    tracker.reset(Rotation2d.fromDegrees(140), positions(9, 0));
    var forward = SwerveDrive.calculateFieldRelativeWheelRequests(
        1, 0, 0, tracker.getPose().getRotation(), 0.8, 0.6);
    assertEquals(0, forward[0].angleRadians(), EPSILON);

    tracker.update(Rotation2d.fromDegrees(230), positions(9, 0));
    var right = SwerveDrive.calculateFieldRelativeWheelRequests(
        1, 0, 0, tracker.getPose().getRotation(), 0.8, 0.6);
    assertEquals(-Math.PI / 2.0, right[0].angleRadians(), EPSILON);
  }

  @Test
  void lostSensorReferenceRequiresResetAndDoesNotCreatePhantomTravel() {
    var tracker = new SwervePoseTracker(0.8, 0.6);
    tracker.update(new Rotation2d(), positions(0, 0));
    tracker.update(new Rotation2d(), positions(2, 0));
    tracker.invalidate();
    tracker.update(Rotation2d.fromDegrees(90), positions(100, 0));
    assertFalse(tracker.isReady());
    assertEquals(2, tracker.getPose().getX(), EPSILON);

    tracker.reset(Rotation2d.fromDegrees(90), positions(100, 0));
    assertTrue(tracker.isReady());
    assertOrigin(tracker.getPose());
    tracker.update(Rotation2d.fromDegrees(90), positions(101, 0));
    assertEquals(1, tracker.getPose().getX(), EPSILON);
  }

  private static SwerveModulePosition[] positions(double meters, double angleDegrees) {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (int i = 0; i < positions.length; i++) {
      positions[i] = new SwerveModulePosition(meters, Rotation2d.fromDegrees(angleDegrees));
    }
    return positions;
  }

  private static void assertOrigin(Pose2d pose) {
    assertEquals(0, pose.getX(), EPSILON);
    assertEquals(0, pose.getY(), EPSILON);
    assertEquals(0, pose.getRotation().getRadians(), EPSILON);
  }
}
