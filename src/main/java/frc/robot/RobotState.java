// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.swerve.DriveConstants;
import org.littletonrobotics.junction.AutoLogOutput;
import org.photonvision.EstimatedRobotPose;

/* based on wpimath/../PoseEstimator.java */
public class RobotState {
  public record OdometryMeasurement(
      SwerveModulePosition[] wheelPositions, Rotation2d gyroAngle, double timestamp) {}

  private static final double poseBufferSizeSeconds = 2; // shorter?

  private static final Vector<N3> STATE_STDS =
      VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));

  private static final Vector<N3> VISION_STDS =
      VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));

  private SwerveDrivePoseEstimator poseEstimator;

  private TimeInterpolatableBuffer<Pose2d> poseBuffer =
      TimeInterpolatableBuffer.createBuffer(poseBufferSizeSeconds);

  private Pose2d odometryPose = new Pose2d(); // motion sensors
  private Pose2d estimatedPose = new Pose2d(); // odometry + vision

  private SwerveModulePosition[] lastWheelPositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private Rotation2d lastGyroAngle = new Rotation2d();

  private static RobotState instance;

  public static RobotState getInstance() {
    if (instance == null) instance = new RobotState();
    return instance;
  }

  private RobotState() {

    poseEstimator =
        new SwerveDrivePoseEstimator(
            DriveConstants.KINEMATICS,
            lastGyroAngle,
            lastWheelPositions,
            new Pose2d(0, 0, lastGyroAngle),
            STATE_STDS,
            VISION_STDS);
  }

  /* update pose estimation based on odometry measurements, based on wpimath */
  public void addOdometryMeasurement(OdometryMeasurement measurement) {
    Twist2d twist =
        DriveConstants.KINEMATICS.toTwist2d(lastWheelPositions, measurement.wheelPositions());
    twist.dtheta = measurement.gyroAngle().minus(lastGyroAngle).getRadians();

    lastWheelPositions = measurement.wheelPositions();
    lastGyroAngle = measurement.gyroAngle();

    // integrate to find difference in pose over time, add to pose estimate
    odometryPose = odometryPose.exp(twist);

    // add post estimate to buffer at timestamp; for vision
    poseBuffer.addSample(measurement.timestamp(), odometryPose);
  }

  // FIXME TO DO
  public void addVisionMeasurement(EstimatedRobotPose pose) {
    // if measurement is old enough to be outside buffer timespan, skip
    if (poseBuffer.getInternalBuffer().isEmpty()
        || poseBuffer.getInternalBuffer().lastKey() < poseBufferSizeSeconds) {
      return;
    }
    // poseBuffer.addSample(pose.timestampSeconds, pose.estimatedPose.toPose2d()); 
    poseEstimator.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds);

  }

  public void resetPose(Pose2d pose) {
    odometryPose = pose;
    estimatedPose = pose;
    poseBuffer.clear();
  }

  @AutoLogOutput(key = "RobotState/OdometryPose")
  public Pose2d getOdometryPose() {
    return odometryPose;
  }

  @AutoLogOutput(key = "RobotState/EstimatedPose")
  public Pose2d getEstimatedPose() {
    return estimatedPose;
  }

}
