package frc.robot.subsystems.swerve.controllers;

import static frc.robot.subsystems.swerve.DriveConstants.DRIVE_CONFIG;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.subsystems.swerve.DriveConstants;
import frc.robot.subsystems.swerve.DriveConstants.DrivebaseConfig;

public class AngleController{
    private double controllerX = 0;
    private double controllerY = 0;
    private double targetRadians = 0;
    //FIXME acceleration
    TrapezoidProfile profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(DRIVE_CONFIG.maxAngularVelocity(), 3));

    public AngleController(double targetDegrees) {
        this.targetRadians = targetDegrees;
    }

    public void acceptJoystickInput(double controllerX, double controllerY, double targetDegrees) {
        this.controllerX = controllerX; 
        this.controllerY = controllerY;
    }



public ChassisSpeeds update(Rotation2d yaw) {


    // eventaully run off of pose estimation?
    return ChassisSpeeds.fromFieldRelativeSpeeds(
        linearVelocity.getX() * DriveConstants.DRIVE_CONFIG.maxLinearVelocity(),
        linearVelocity.getY() * DriveConstants.DRIVE_CONFIG.maxLinearVelocity(),
        omega * DriveConstants.DRIVE_CONFIG.maxAngularVelocity(),
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
    PIDController rotController = new PIDController(Math.toRadians(0.0179), 0, 0); //FIXME P is tuned for degrees not radians
    Translation2d rotationalVelocity = calculateLinearVelocity(controllerX, controllerY);
    TrapezoidProfile.State target = profile.calculate(0.2, 
        new TrapezoidProfile.State(currentYaw.getRadians(), currentVelocity), 
        new TrapezoidProfile.State(targetRadians, 0));
    return rotController.calculate(target.position);
  }
}

