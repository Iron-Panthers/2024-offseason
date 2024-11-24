package frc.robot.subsystems.swerve.controllers;

import static frc.robot.subsystems.swerve.DriveConstants.DRIVE_CONFIG;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.swerve.DriveConstants;
import frc.robot.subsystems.swerve.DriveConstants.DrivebaseConfig;

public class AngleController{
    private double controllerX = 0;
    private double controllerY = 0;
    PIDController rotController = new PIDController(0.179,0,0); //FIXME!!

    public AngleController(double targetDegrees) {
        rotController.setSetpoint(targetDegrees);
    }

    public void acceptJoystickInput(double controllerX, double controllerY) {
        this.controllerX = controllerX; 
        this.controllerY = controllerY;
    }



public ChassisSpeeds update(Rotation2d yaw) {

    Translation2d linearVelocity = calculateLinearVelocity(controllerX, controllerY);
    return ChassisSpeeds.fromFieldRelativeSpeeds(
        linearVelocity.getX() * DriveConstants.DRIVE_CONFIG.maxLinearVelocity(),
        linearVelocity.getY() * DriveConstants.DRIVE_CONFIG.maxLinearVelocity(),
        calculateRotationalVelocity(yaw) * DriveConstants.DRIVE_CONFIG.maxAngularVelocity(),
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

  public double calculateRotationalVelocity(Rotation2d currentYaw){
    return rotController.calculate(currentYaw.getRadians());
  }
}

