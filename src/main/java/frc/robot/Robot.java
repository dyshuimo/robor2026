// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.subsystems.swerve.SwerveDrive;


public class Robot extends TimedRobot {
  private final PS4Controller controller = new PS4Controller(Constants.PS4_PORT);
  private final SwerveDrive swerve = new SwerveDrive();

  @Override
  public void robotInit() {
    swerve.resetForStartup();
  }

  @Override
  public void robotPeriodic() {
    swerve.periodic();
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
  public void teleopInit() {
    swerve.prepareForTeleop();
  }

  @Override
  public void teleopPeriodic() {
    if (!controller.isConnected()) {
      swerve.stop();
      return;
    }

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
  public void testInit() {
    swerve.stop();
  }
}
