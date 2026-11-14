// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.subsystems.swerve.SwerveDrive;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private final PS4Controller controller = new PS4Controller(Constants.PS4_PORT);
  private final SwerveDrive swerve = new SwerveDrive();

  @Override
  public void robotPeriodic() {
    swerve.periodic();
    // Available in Teleop or while disabled; independent of joystick movement.
    if (controller.isConnected() && (isTeleop() || isDisabled())
        && controller.getTriangleButtonPressed()) {
      swerve.resetOdometry();
    }
  }

  @Override
  public void autonomousInit() {
    swerve.stop();
  }

  @Override
  public void teleopPeriodic() {
    if (!controller.isConnected()) {
      swerve.stop();
      return;
    }

    // Left stick translates anywhere on the field; right stick X spins the robot.
    // WPILib convention: forward, left, and counterclockwise are positive.
    double forward = MathUtil.applyDeadband(-controller.getLeftY(), Constants.STICK_DEADBAND);
    double left = MathUtil.applyDeadband(-controller.getLeftX(), Constants.STICK_DEADBAND);
    double turn = MathUtil.applyDeadband(-controller.getRightX(), Constants.STICK_DEADBAND);
    swerve.drive(forward, left, turn);
  }

  @Override
  public void disabledInit() {
    swerve.stop();
  }

  @Override
  public void disabledPeriodic() {
    // With the robot disabled, manually point ALL modules forward first.
    if (controller.isConnected() && controller.getOptionsButtonPressed()) {
      swerve.zeroSteering();
    }
  }

  @Override
  public void testInit() {
    swerve.stop();
  }
}
