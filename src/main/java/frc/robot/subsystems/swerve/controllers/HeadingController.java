package frc.robot.subsystems.swerve.controllers;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.subsystems.swerve.DriveConstants;
import org.littletonrobotics.junction.Logger;

public class HeadingController {

  private ProfiledPIDController controller;
  private Rotation2d targetRotation2d;

  public HeadingController(Rotation2d tRotation2d) {
    controller =
        new ProfiledPIDController(
            4,
            0,
            0,
            new TrapezoidProfile.Constraints(
                DriveConstants.DRIVE_CONFIG.maxAngularVelocity(),
                DriveConstants.DRIVE_CONFIG.maxAngularAcceleration()));
    targetRotation2d = tRotation2d;
  }

  public void setTarget(Rotation2d target) {
    targetRotation2d = target;
  }

  public void changeTarget(double delta) {
    targetRotation2d = new Rotation2d(targetRotation2d.getRadians() + delta);
  }

  public Rotation2d update(Rotation2d yaw, double currentRotationalVelocity) {
    Logger.recordOutput("Swerve/TargetHeading", normalizeRadians(targetRotation2d.getRadians()));

    return new Rotation2d(
        controller.calculate(
            calculateRelativeAngularDifference(yaw.getRadians(), targetRotation2d.getRadians())));
  }

  private double calculateRelativeAngularDifference(double currentAngle, double targetAngle) {
    double a = normalizeRadians(currentAngle - targetAngle);
    double b = normalizeRadians(targetAngle - currentAngle);
    return a < b ? a : -b;
  }

  public double normalizeRadians(double radians) {
    return (radians % (2 * Math.PI) + (2 * Math.PI)) % (2 * Math.PI);
  }
}
