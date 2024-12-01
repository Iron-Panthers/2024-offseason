package frc.robot.subsystems.swerve.controllers;

import static frc.robot.subsystems.swerve.DriveConstants.DRIVE_CONFIG;

import org.w3c.dom.ls.LSException;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.subsystems.swerve.Drive;
import frc.robot.subsystems.swerve.DriveConstants;
import frc.robot.subsystems.swerve.DriveConstants.DrivebaseConfig;

public class HeadingController{
  private double controllerX = 0;
  private double controllerY = 0;
  private ProfiledPIDController controller;
  private Rotation2d targetRotation2d;

  public HeadingController(Rotation2d tRotation2d) {
    ProfiledPIDController controller = new ProfiledPIDController(
      1, 0, 0,
      new TrapezoidProfile.Constraints(DriveConstants.DRIVE_CONFIG.maxAngularVelocity(), DriveConstants.DRIVE_CONFIG.maxAngularAcceleration()));
      targetRotation2d = tRotation2d;
  }

  public void acceptJoystickInput(double controllerX, double controllerY) {
      this.controllerX = controllerX; 
      this.controllerY = controllerY;
  }

  public void setTarget(Rotation2d target){
    targetRotation2d = target;
  }

  public void changeTarget(double delta){
    targetRotation2d = new Rotation2d(targetRotation2d.getRadians() + delta);
  }

  public ChassisSpeeds update(Rotation2d yaw, double currentRotationalVelocity) {

    Translation2d linearVelocity = calculateLinearVelocity(controllerX, controllerY);
    return ChassisSpeeds.fromFieldRelativeSpeeds(
        linearVelocity.getX() * DriveConstants.DRIVE_CONFIG.maxLinearVelocity(),
        linearVelocity.getY() * DriveConstants.DRIVE_CONFIG.maxLinearVelocity(),
        calculateRotationalVelocity(yaw, currentRotationalVelocity) * DriveConstants.DRIVE_CONFIG.maxAngularVelocity(),
        yaw);
  }

  public Translation2d calculateLinearVelocity(double x, double y) {
    // apply deadband, raise magnitude to exponent
    double magnitude = MathUtil.applyDeadband(Math.hypot(x, y), 0.1);
    magnitude = Math.pow(magnitude, 1.5);

    Rotation2d theta = new Rotation2d(x, y);

    Translation2d linearVelocity =
        new Pose2d(new Translation2d(), theta)
            .transformBy(new Transform2d(magnitude, 0, new Rotation2d()))
            .getTranslation();
    return linearVelocity;
  }

  public double calculateRotationalVelocity(Rotation2d currentYaw, double currentVelocity){
    return controller.calculate(currentYaw.getRadians(), new TrapezoidProfile.State(targetRotation2d.getRadians(), 0));
  }
}

