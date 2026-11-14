package frc.robot.subsystems.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.jupiter.api.Test;

class SwerveDriveTest {
  private static final double EPSILON = 1e-9;

  @Test
  void forwardPointsAllWheelsForward() {
    var requests = SwerveDrive.calculateWheelRequests(0.5, 0, 0, 0.8, 0.6);
    for (var request : requests) {
      assertEquals(0.5, request.speed(), EPSILON);
      assertEquals(0, request.angleRadians(), EPSILON);
    }
  }

  @Test
  void strafeLeftPointsAllWheelsLeft() {
    var requests = SwerveDrive.calculateWheelRequests(0, 0.5, 0, 0.8, 0.6);
    for (var request : requests) {
      assertEquals(0.5, request.speed(), EPSILON);
      assertEquals(Math.PI / 2.0, request.angleRadians(), EPSILON);
    }
  }

  @Test
  void reversePointsAllWheelsBackward() {
    var requests = SwerveDrive.calculateWheelRequests(-0.5, 0, 0, 0.8, 0.6);
    for (var request : requests) {
      assertEquals(0.5, request.speed(), EPSILON);
      assertEquals(Math.PI, Math.abs(request.angleRadians()), EPSILON);
    }
  }

  @Test
  void counterclockwiseRotationIsTangentAtEveryCornerOfRectangle() {
    var requests = SwerveDrive.calculateWheelRequests(0, 0, 1, 0.8, 0.6);
    double[] expectedForward = {-0.6, 0.6, -0.6, 0.6};
    double[] expectedLeft = {0.8, 0.8, -0.8, -0.8};
    for (int i = 0; i < requests.length; i++) {
      assertEquals(1.0, requests[i].speed(), EPSILON);
      assertEquals(expectedForward[i], Math.cos(requests[i].angleRadians()), EPSILON);
      assertEquals(expectedLeft[i], Math.sin(requests[i].angleRadians()), EPSILON);
    }
  }

  @Test
  void combinedMotionScalesAllVectorsTogetherWithoutClippingTheirDirections() {
    var requests = SwerveDrive.calculateWheelRequests(0.6, -0.4, 0.7, 0.8, 0.6);
    double[] expectedForward = {0.18, 1.02, 0.18, 1.02};
    double[] expectedLeft = {0.16, 0.16, -0.96, -0.96};
    double scale = Math.hypot(1.02, 0.96);
    for (int i = 0; i < requests.length; i++) {
      var request = requests[i];
      assertTrue(request.speed() <= 1.0);
      assertEquals(expectedForward[i] / scale,
          request.speed() * Math.cos(request.angleRadians()), EPSILON);
      assertEquals(expectedLeft[i] / scale,
          request.speed() * Math.sin(request.angleRadians()), EPSILON);
    }
  }

  @Test
  void centeredSticksRequestNoMovement() {
    var requests = SwerveDrive.calculateWheelRequests(0, 0, 0, 0.8, 0.6);
    for (var request : requests) {
      assertEquals(0, request.speed(), EPSILON);
      assertTrue(Double.isFinite(request.angleRadians()));
    }
  }

  @Test
  void fieldForwardAtNinetyDegreesCommandsRobotRight() {
    var requests = SwerveDrive.calculateFieldRelativeWheelRequests(
        0.5, 0, 0, Rotation2d.fromDegrees(90), 0.8, 0.6);
    for (var request : requests) {
      assertEquals(0.5, request.speed(), EPSILON);
      assertEquals(-Math.PI / 2.0, request.angleRadians(), EPSILON);
    }
  }

  @Test
  void fieldForwardWhenFacingBackwardCommandsRobotBackward() {
    var requests = SwerveDrive.calculateFieldRelativeWheelRequests(
        0.5, 0, 0, Rotation2d.fromDegrees(180), 0.8, 0.6);
    for (var request : requests) {
      assertEquals(0.5, request.speed(), EPSILON);
      assertEquals(Math.PI, Math.abs(request.angleRadians()), EPSILON);
    }
  }

  @Test
  void diagonalFieldInputMaintainsItsFieldDirectionAtAnArbitraryHeading() {
    var heading = Rotation2d.fromDegrees(37);
    var requests = SwerveDrive.calculateFieldRelativeWheelRequests(
        0.3, -0.4, 0, heading, 0.8, 0.6);
    for (var request : requests) {
      assertEquals(0.5, request.speed(), EPSILON);
      double fieldAngle = request.angleRadians() + heading.getRadians();
      assertEquals(0.3, request.speed() * Math.cos(fieldAngle), EPSILON);
      assertEquals(-0.4, request.speed() * Math.sin(fieldAngle), EPSILON);
    }
  }

  @Test
  void rightStickSpinIsIndependentOfFieldHeading() {
    var expected = SwerveDrive.calculateWheelRequests(0, 0, -0.7, 0.8, 0.6);
    var actual = SwerveDrive.calculateFieldRelativeWheelRequests(
        0, 0, -0.7, Rotation2d.fromDegrees(123), 0.8, 0.6);
    for (int i = 0; i < actual.length; i++) {
      assertEquals(expected[i].speed(), actual[i].speed(), EPSILON);
      assertEquals(expected[i].angleRadians(), actual[i].angleRadians(), EPSILON);
    }
  }
}
